import asyncio
import math
from uuid import uuid4

import pytest

from prag_rover.assistant import AssistantBridge
from prag_rover.controller import SimulatedController, State
from prag_rover.protocol import Request
from prag_rover.runtime import RoverRuntime


class LogicalTime:
    def __init__(self):
        self.at = 0.0
        self.sleepers = []

    def __call__(self):
        return self.at

    async def sleep(self, delay):
        future = asyncio.get_running_loop().create_future()
        entry = (self.at + delay, future)
        self.sleepers.append(entry)
        try:
            await future
        finally:
            self.sleepers.remove(entry)

    async def advance(self, duration):
        self.at += duration
        for deadline, future in tuple(self.sleepers):
            if deadline <= self.at and not future.done():
                future.set_result(None)
        # Run newly-ready work without advancing real time.
        for _ in range(5):
            await asyncio.sleep(0)

    async def finish(self, task):
        for _ in range(1000):
            if task.done():
                return await task
            await self.advance(0.01)
        raise AssertionError("Logical task did not finish within its budget")


async def call(runtime, operation, **arguments):
    return await runtime.dispatch(
        Request(uuid4().hex, operation, "operator", arguments)
    )


def test_disconnect_expires_locally_and_never_resumes_on_reconnect():
    async def scenario():
        clock = LogicalTime()
        controller = SimulatedController(now=clock)
        async with RoverRuntime(controller=controller, sleep=clock.sleep) as runtime:
            await call(runtime, "arm")
            job = await call(runtime, "move", distance_m=1)
            await clock.advance(0)
            assert controller.state == State.MOVING
            await call(runtime, "link", connected=False)
            assert runtime.jobs[job["id"]]["state"] == "cancelled"
            stop = await call(runtime, "stop")
            assert stop["stop_delivered"] is False
            assert controller.left > 0  # A broken link did not deliver a stop.
            await clock.advance(0.5)
            assert controller.fault == "command_timeout"
            assert controller.left == controller.right == 0
            await call(runtime, "link", connected=True)
            assert controller.state == State.FAULT
            await call(runtime, "reset")
            assert controller.state == State.DISARMED
            assert runtime.active is None

    asyncio.run(scenario())


def test_reverse_turn_and_demo_mission_under_logical_time():
    async def scenario():
        clock = LogicalTime()
        controller = SimulatedController(now=clock)
        async with RoverRuntime(controller=controller, sleep=clock.sleep) as runtime:
            await call(runtime, "arm")
            job = await call(runtime, "move", distance_m=-0.1)
            await clock.finish(runtime.active[1])
            assert runtime.jobs[job["id"]]["state"] == "completed"
            assert controller.x == pytest.approx(-0.1, abs=0.002)
            await call(runtime, "arm")
            await call(runtime, "turn", angle_deg=90)
            await clock.finish(runtime.active[1])
            assert controller.heading == pytest.approx(math.pi / 2, abs=0.03)
            await call(runtime, "arm")
            job = await call(runtime, "mission", name="desk-demo")
            await clock.finish(runtime.active[1])
            result = runtime.jobs[job["id"]]
            assert result["state"] == "completed"
            assert result["result"]["camera_source"] == "rover.synthetic"

    asyncio.run(scenario())


def test_supervisor_failure_stops_motion_and_surfaces_on_context_exit():
    async def scenario():
        clock = LogicalTime()

        async def failing_sleep(delay):
            await asyncio.sleep(0)
            raise RuntimeError("injected scheduler failure")

        runtime = RoverRuntime(
            controller=SimulatedController(now=clock), sleep=failing_sleep
        )
        with pytest.raises(RuntimeError, match="supervision failed"):
            async with runtime:
                await call(runtime, "arm")
                job = await call(runtime, "move", distance_m=1)
                await runtime.shutdown.wait()
                assert runtime.controller.left == runtime.controller.right == 0
                assert runtime.controller.fault == "supervisor_failure"
                with pytest.raises(RuntimeError, match="supervision failed"):
                    await call(runtime, "arm")
        assert runtime.active is None
        assert runtime.jobs[job["id"]]["state"] in ("cancelled", "failed")
        assert runtime._supervisor.done()

    asyncio.run(scenario())


def test_bumper_and_shutdown_cancel_pending_image_acquisition():
    class BlockingCamera:
        def __init__(self):
            self.started = asyncio.Event()
            self.closed = asyncio.Event()

        async def read(self):
            self.started.set()
            try:
                await asyncio.Event().wait()
            finally:
                self.closed.set()

    async def scenario():
        camera = BlockingCamera()
        async with RoverRuntime(assistant=AssistantBridge(camera=camera)) as runtime:
            await call(runtime, "arm")
            job = await call(runtime, "mission", name="look")
            await camera.started.wait()
            await call(runtime, "bumper", pressed=True)
            assert camera.closed.is_set()
            assert runtime.jobs[job["id"]]["state"] == "cancelled"
            assert runtime.controller.state == State.FAULT
        assert runtime._supervisor.done()

    asyncio.run(scenario())


def test_independent_inspection_reserves_stationary_state():
    from prag_rover.assistant import SyntheticCamera

    class Camera:
        def __init__(self):
            self.started, self.release = asyncio.Event(), asyncio.Event()

        async def read(self):
            self.started.set()
            await self.release.wait()
            return await SyntheticCamera().read()

    async def scenario():
        camera = Camera()
        async with RoverRuntime(assistant=AssistantBridge(camera=camera)) as runtime:
            await call(runtime, "arm")
            inspection = asyncio.create_task(call(runtime, "inspect"))
            await camera.started.wait()
            with pytest.raises(ValueError, match="stationary inspection"):
                await call(runtime, "move", distance_m=0.1)
            await call(runtime, "stop")
            assert runtime.controller.left == 0
            camera.release.set()
            await inspection
            assert runtime._stationary_inspections == 0
            await call(runtime, "arm")

    asyncio.run(scenario())


def test_provider_failure_fails_mission_without_fixture_fallback():
    from pragyantra.application.interfaces import ProviderError

    class FailedInspection:
        async def evaluate(self, frame, questions):
            raise ProviderError("injected failure")

    async def scenario():
        async with RoverRuntime(
            assistant=AssistantBridge(inspection=FailedInspection())
        ) as runtime:
            await call(runtime, "arm")
            job = await call(runtime, "mission", name="look")
            await runtime.active[1]
            record = runtime.jobs[job["id"]]
            assert record["state"] == "failed"
            assert record["result"]["judgment"]["status"] == "provider_failure"
            assert runtime.controller.state == State.DISARMED

    asyncio.run(scenario())
