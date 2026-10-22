"""One rover owner, cancellable jobs, and independent simulated supervision."""

import asyncio
import inspect as python_inspect
import math
from collections import OrderedDict
from hashlib import sha256
from pathlib import Path
from uuid import uuid4

from prag_rover.assistant import AssistantBridge
from prag_rover.controller import SimulatedController, State
from prag_rover.protocol import Request, fields, number


class RoverRuntime:
    def __init__(
        self,
        *,
        controller=None,
        assistant=None,
        allow_agent_motion=False,
        artifacts=None,
        sleep=asyncio.sleep,
    ):
        self.controller = controller or SimulatedController()
        self.assistant = assistant or AssistantBridge()
        self.allow_agent_motion = allow_agent_motion
        self.artifacts = Path(artifacts) if artifacts is not None else None
        self.owner = None
        self.link = True
        self.jobs = OrderedDict()
        self.active = None
        self.shutdown = asyncio.Event()
        self._supervisor = None
        self._stopping = 0
        self._sleep = sleep
        self._failure = None
        self._stationary_inspections = 0

    async def __aenter__(self):
        if hasattr(self.controller, "start"):
            await self.controller.start()
        self._supervisor = asyncio.create_task(self._supervise())
        return self

    async def _control(self, name, *args, **kwargs):
        result = getattr(self.controller, name)(*args, **kwargs)
        if python_inspect.isawaitable(result):
            return await result
        return result

    async def __aexit__(self, *args):
        try:
            await self.stop()
        finally:
            self._supervisor.cancel()
            await asyncio.gather(self._supervisor, return_exceptions=True)
            if hasattr(self.controller, "close"):
                await self.controller.close()
        if self._failure is not None and args[0] is None:
            raise RuntimeError("Controller supervision failed") from self._failure

    async def _supervise(self):
        try:
            while True:
                await self._control("tick")
                if self.controller.state == State.FAULT and self.active is not None:
                    await self.stop()
                await self._sleep(0.02)
        except Exception as exc:
            # A dead supervisor must not leave the session running unnoticed.
            self._failure = exc
            if self.controller.simulated:
                self.controller.stop("supervisor_failure")
            else:
                self.link = False
                await self.controller.close()
            if self.active is not None:
                self.active[1].cancel()
            self.shutdown.set()

    def status(self):
        return {
            "controller": self.controller.snapshot(),
            "owner": self.owner,
            "controller_link": self.link,
            "active_job": self.active[0] if self.active else None,
            "agent_motion_enabled": self.allow_agent_motion,
        }

    async def stop(self):
        # Reject rearm while an old job is still unwinding. Stop may be called
        # concurrently; a counter keeps the barrier until every stop finishes.
        self._stopping += 1
        try:
            if self._failure is None:
                try:
                    await self._control("tick")
                except Exception as exc:
                    self._failure = exc
                    if self.controller.simulated:
                        self.controller.stop("supervisor_failure")
                    else:
                        self.link = False
                    self.shutdown.set()
            if self.link:
                try:
                    await self._control("stop")
                except (OSError, TimeoutError, RuntimeError, ValueError):
                    self.link = False
            running, self.active = self.active, None
            self.owner = None
            if running is not None:
                identifier, task = running
                task.cancel()
                await asyncio.gather(task, return_exceptions=True)
                if self.jobs[identifier]["state"] == "running":
                    self.jobs[identifier]["state"] = "cancelled"
        finally:
            self._stopping -= 1

    def _motion_allowed(self, owner):
        if self._stopping:
            raise ValueError("Wait for the previous stop to finish")
        if self._stationary_inspections:
            raise ValueError("Wait for the stationary inspection to finish")
        if not self.link:
            raise ValueError("Controller link is unavailable")
        if owner.startswith("agent:") and not self.allow_agent_motion:
            raise ValueError("Agent motion requires explicit session opt-in")
        if self.owner != owner:
            raise ValueError("Motion requires the current owner's armed session")
        if self.controller.state not in (State.READY, State.MOVING):
            raise ValueError("Arm the controller before motion")
        if self.active is not None:
            raise ValueError("A motion job already owns the controller")

    async def _segment(self, distance, angle, speed):
        if self.controller.geometry is None:
            raise ValueError("Controller calibration is missing")
        extent = distance if distance else angle * self.controller.geometry.track_m / 2
        if not extent:
            return
        start = self.controller.left_distance
        direction = math.copysign(1, extent)
        left = speed * direction
        right = left if distance else -left
        # Positive angles are counterclockwise: left backward, right forward.
        if angle:
            left, right = -left, -right
            start = self.controller.right_distance
        budget_end = self.controller.now() + abs(extent) / speed * 3 + 2
        while True:
            if not self.link:
                raise ValueError("Controller link lost during motion")
            if self.controller.state == State.FAULT:
                raise ValueError(f"Controller fault: {self.controller.fault}")
            await self._control("tick")
            position = (
                self.controller.left_distance
                if distance
                else self.controller.right_distance
            )
            remaining_distance = abs(extent) - direction * (position - start)
            if remaining_distance <= 0:
                break
            if self.controller.now() >= budget_end:
                raise ValueError(
                    "Motion made insufficient progress before its deadline"
                )
            await self._control(
                "drive",
                left,
                right,
                sequence=self.controller.sequence + 1,
                expires_at=self.controller.now() + 0.299,
            )
            await self._sleep(min(0.05, remaining_distance / speed))
        # The terminal sample may reach lease expiry before the host wakes.
        # Normal segment completion uses a lease beyond the intended stop time;
        # the host stops at the deadline, while an absent host expires locally.
        await self._control("finish_segment")
        traveled = (
            self.controller.left_distance
            if distance
            else self.controller.right_distance
        ) - start
        return {"wheel_travel_m": traveled, "simulated": self.controller.simulated}

    def _start_job(self, owner, name, work):
        self._motion_allowed(owner)
        identifier = uuid4().hex[:12]
        while len(self.jobs) >= 32:
            self.jobs.popitem(last=False)
        record = {"id": identifier, "name": name, "owner": owner, "state": "running"}
        self.jobs[identifier] = record

        async def execute():
            try:
                record["result"] = await work()
                record["state"] = "completed"
                if (
                    isinstance(record["result"], dict)
                    and "judgment" in record["result"]
                ):
                    outcome = record["result"]["judgment"]["status"]
                    if outcome != "completed":
                        record["state"] = "failed"
                        record["error"] = f"Inspection returned {outcome}"
            except asyncio.CancelledError:
                record["state"] = "cancelled"
                raise
            except Exception as exc:
                record["state"] = "failed"
                record["error"] = str(exc)
            finally:
                try:
                    if self.link:
                        if self._failure is None:
                            await self._control("tick")
                        await self._control("stop")
                except (OSError, TimeoutError, RuntimeError, ValueError) as exc:
                    self.link = False
                    record["state"] = "failed"
                    record["error"] = f"Stop was not acknowledged: {exc}"
                finally:
                    if self.active is not None and self.active[0] == identifier:
                        self.active = None
                        self.owner = None

        task = asyncio.create_task(execute())
        self.active = (identifier, task)
        return dict(record)

    async def dispatch(self, request: Request):
        op, args, owner = request.operation, request.arguments, request.owner
        if self._failure is not None:
            raise RuntimeError("Controller supervision failed; restart the session")
        if op in (
            "status",
            "jobs",
            "arm",
            "disarm",
            "stop",
            "reset",
            "takeover",
            "shutdown",
        ):
            fields(args)
        if op == "status":
            return self.status()
        if op == "jobs":
            return list(self.jobs.values())
        if op in ("stop", "disarm"):
            await self.stop()
            return {"stop_delivered": self.link, **self.status()}
        if op in ("reset", "takeover", "shutdown") and owner != "operator":
            raise ValueError("Only the operator can reset, take over, or shut down")
        if op == "shutdown":
            await self.stop()
            return {"shutdown": True}
        if op == "takeover":
            await self.stop()
            return self.status()
        if op == "reset":
            await self.stop()
            if not self.link:
                raise ValueError("Restore the controller link before reset")
            await self._control("reset")
            return self.status()
        if op == "arm":
            if self._stopping:
                raise ValueError("Wait for the previous stop to finish")
            if not self.link:
                raise ValueError("Controller link is unavailable")
            if owner.startswith("agent:") and not self.allow_agent_motion:
                raise ValueError("Agent motion requires explicit session opt-in")
            if self.owner is not None:
                raise ValueError(
                    "Controller already has an owner; stop or take over first"
                )
            await self._control("arm")
            self.owner = owner
            return self.status()
        if op == "bumper":
            if not self.controller.simulated:
                raise ValueError("Bumper injection is only available in simulation")
            fields(args, ("pressed",))
            if owner != "operator":
                raise ValueError("Only the operator may inject simulated faults")
            self.controller.set_bumper(args["pressed"])
            if args["pressed"]:
                await self.stop()
            return self.status()
        if op == "link":
            if not self.controller.simulated:
                raise ValueError("Link injection is only available in simulation")
            fields(args, ("connected",))
            if owner != "operator" or type(args["connected"]) is not bool:
                raise ValueError("Only the operator can set a boolean simulated link")
            self.link = args["connected"]
            # Stop renewal when disconnected; deliver a stop on reconnection.
            # Neither path resumes a cancelled motion or mission.
            await self.stop()
            return self.status()
        if op == "drive":
            fields(args, ("left_mps", "right_mps"), ("lease_s",))
            if owner != "operator":
                raise ValueError(
                    "Agents may run named missions, not raw wheel commands"
                )
            self._motion_allowed(owner)
            lease = number(
                args.get("lease_s", 0.3),
                name="lease",
                minimum=0.01,
                maximum=self.controller.MAX_LEASE,
            )
            await self._control(
                "drive",
                args["left_mps"],
                args["right_mps"],
                sequence=self.controller.sequence + 1,
                expires_at=self.controller.now() + lease,
            )
            return self.status()
        if op in ("move", "turn"):
            fields(
                args,
                ("distance_m",) if op == "move" else ("angle_deg",),
                ("speed_mps",),
            )
            if owner != "operator":
                raise ValueError(
                    "Agents may run named missions, not arbitrary movements"
                )
            speed = number(
                args.get("speed_mps", 0.1), name="speed", minimum=0.03, maximum=0.15
            )
            distance = (
                number(args["distance_m"], name="distance", minimum=-1, maximum=1)
                if op == "move"
                else 0
            )
            angle = (
                math.radians(
                    number(args["angle_deg"], name="angle", minimum=-180, maximum=180)
                )
                if op == "turn"
                else 0
            )
            return self._start_job(
                owner, op, lambda: self._segment(distance, angle, speed)
            )
        if op == "mission":
            fields(args, ("name",))
            if args["name"] not in ("look", "desk-demo"):
                raise ValueError("Available missions: look, desk-demo")

            async def mission():
                if args["name"] == "desk-demo":
                    await self._segment(0.05, 0, 0.1)
                    await self._segment(0, math.radians(15), 0.1)
                return await self.assistant.inspect(
                    "Is a distinct object visible?",
                    describe=True,
                )

            return self._start_job(owner, args["name"], mission)
        if op == "ask":
            fields(args, ("text",))
            text = args["text"]
            if not isinstance(text, str) or not text.strip() or len(text) > 4000:
                raise ValueError("Text requires 1–4000 characters")
            if text.strip() == "/stop":
                await self.stop()
                return {"stop_delivered": self.link, **self.status()}
            self.assistant.update_telemetry(self.controller.snapshot())
            return await self.assistant.ask(text)
        if op == "inspect":
            fields(args, (), ("question", "describe"))
            if self.controller.state == State.MOVING or self.active is not None:
                raise ValueError("Stop motion before an independent inspection")
            question = args.get("question", "Is a distinct object visible?")
            # Validate before any image acquisition or provider call.
            from pragyantra.application.inspection import InspectionQuestion

            InspectionQuestion("inspection", question)
            if type(args.get("describe", False)) is not bool:
                raise ValueError("Describe must be boolean")
            self._stationary_inspections += 1
            try:
                return await self.assistant.inspect(
                    question,
                    describe=args.get("describe", False),
                )
            finally:
                self._stationary_inspections -= 1
        if op == "capture":
            fields(args)
            if self.artifacts is None:
                raise ValueError("Capture storage is not configured")
            frame = await self.assistant.camera.read()
            self.artifacts.mkdir(parents=True, exist_ok=True)
            path = self.artifacts / f"{uuid4().hex}.jpg"
            with path.open("xb") as output:
                output.write(frame.data)
            return {
                "path": str(path),
                "source": frame.source,
                "acquired_at": frame.observed_at.isoformat(),
                "evidence_id": "sha256:" + sha256(frame.data).hexdigest(),
            }
        raise ValueError("Operation is not implemented")
