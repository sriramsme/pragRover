"""CLI-first rover host, suitable for an SSH terminal and JSON agent calls."""

import argparse
import asyncio
import json
import os
import sys
from contextlib import AsyncExitStack
from pathlib import Path
from uuid import uuid4

from prag_rover.assistant import AssistantBridge, FileCamera, HttpSnapshotCamera
from prag_rover.ipc import LocalServer, record, send
from prag_rover.protocol import Request
from prag_rover.runtime import RoverRuntime


def parser():
    result = argparse.ArgumentParser(
        description="pragRover: ESP32 firmware host and offline simulator"
    )
    result.add_argument("--socket", type=Path, default=Path("data/rover.sock"))
    result.add_argument(
        "--owner", default="operator", help="operator or agent:NAME (local attribution)"
    )
    result.add_argument(
        "--json", action="store_true", help="Emit one compact JSON object"
    )
    commands = result.add_subparsers(dest="command", required=True)
    session = commands.add_parser(
        "session", help="Own a physical or simulated rover until shutdown/Ctrl+C"
    )
    controller = session.add_mutually_exclusive_group(required=True)
    controller.add_argument("--simulated", action="store_true")
    controller.add_argument(
        "--controller", metavar="HOST:PORT", help="Connect motion firmware explicitly"
    )
    session.add_argument("--controller-token-env", default="PRAG_ROVER_TOKEN")
    session.add_argument("--controller-timeout", type=float, default=0.2)
    session.add_argument("--allow-agent-motion", action="store_true")
    session.add_argument(
        "--live", action="store_true", help="Opt into paid OpenAI calls on requests"
    )
    session.add_argument(
        "--interpret",
        choices=("fixture", "typesafe"),
        default="fixture",
        help="typesafe explicitly opts into paid Jev interpretation",
    )
    session.add_argument("--model", default="gpt-5.6-luna")
    session.add_argument("--decision-model", default="gpt-6-luna")
    session.add_argument("--jev-model", default="jev-1.14.0")
    camera = session.add_mutually_exclusive_group()
    camera.add_argument(
        "--image", type=Path, help="Read a file; timestamp means acquisition"
    )
    camera.add_argument(
        "--camera-url", help="Trusted configured HTTP snapshot endpoint"
    )
    session.add_argument("--camera-token-env", default="PRAG_CAMERA_TOKEN")
    session.add_argument("--artifacts", type=Path, default=Path("data/captures"))
    commands.add_parser(
        "demo", help="Offline end-to-end simulation; no session required"
    )
    for name in (
        "status",
        "jobs",
        "arm",
        "stop",
        "disarm",
        "reset",
        "takeover",
        "shutdown",
        "capture",
    ):
        commands.add_parser(name)
    drive = commands.add_parser(
        "drive", help="One short wheel command; repeat to renew"
    )
    drive.add_argument("--left", required=True, type=float)
    drive.add_argument("--right", required=True, type=float)
    drive.add_argument("--lease", type=float, default=0.3)
    move = commands.add_parser("move")
    move.add_argument("distance", type=float, help="Meters, within [-1, 1]")
    move.add_argument("--speed", type=float, default=0.1)
    turn = commands.add_parser("turn")
    turn.add_argument("angle", type=float, help="Degrees; positive is counterclockwise")
    turn.add_argument("--speed", type=float, default=0.1)
    mission = commands.add_parser("mission")
    mission.add_argument("name", choices=("look", "desk-demo"))
    ask = commands.add_parser("ask")
    ask.add_argument("text")
    inspect = commands.add_parser("inspect")
    inspect.add_argument("--question", default="Is a distinct object visible?")
    inspect.add_argument(
        "--describe", action="store_true", help="Also generate a caption"
    )
    for name in ("bumper", "link"):
        fault = commands.add_parser(name, help="Inject simulated state")
        fault.add_argument("state", choices=("on", "off"))
    call = commands.add_parser(
        "call", help="Versioned machine command; no shell evaluation"
    )
    call.add_argument("operation")
    call.add_argument("--arguments", default="{}", help="JSON object")
    call.add_argument("--id", default=None, help="Reuse only for an identical retry")
    return result


def request_for(args):
    op = args.command
    payload = {}
    if op == "drive":
        payload = {
            "left_mps": args.left,
            "right_mps": args.right,
            "lease_s": args.lease,
        }
    elif op == "move":
        payload = {"distance_m": args.distance, "speed_mps": args.speed}
    elif op == "turn":
        payload = {"angle_deg": args.angle, "speed_mps": args.speed}
    elif op == "mission":
        payload = {"name": args.name}
    elif op == "ask":
        payload = {"text": args.text}
    elif op == "inspect":
        payload = {"question": args.question, "describe": args.describe}
    elif op == "bumper":
        payload = {"pressed": args.state == "on"}
    elif op == "link":
        payload = {"connected": args.state == "on"}
    elif op == "call":
        op, payload = args.operation, json.loads(args.arguments)
    return Request(getattr(args, "id", None) or uuid4().hex, op, args.owner, payload)


async def serve(args):
    async with AsyncExitStack() as stack:
        settings = {}
        if args.image:
            settings["camera"] = FileCamera(args.image)
        elif args.camera_url:
            settings["camera"] = HttpSnapshotCamera(
                args.camera_url,
                token=os.environ.get(args.camera_token_env),
            )
        controller = None
        if args.controller:
            from prag_rover.device import DeviceController

            host, separator, port = args.controller.rpartition(":")
            if not separator:
                raise ValueError("Controller endpoint must be HOST:PORT")
            controller = DeviceController(
                host,
                int(port),
                os.environ.get(args.controller_token_env),
                timeout=args.controller_timeout,
            )
        if args.live:
            from pragyantra.providers.openai_decisions import openai_decisions
            from pragyantra.providers.openai_services import openai_services

            services = await stack.enter_async_context(
                openai_services(model=args.model)
            )
            decisions = await stack.enter_async_context(
                openai_decisions(model=args.decision_model)
            )
            settings.update(
                conversation=services,
                vision=services.vision,
                inspection=decisions,
                live=True,
            )
        if args.interpret == "typesafe":
            from pragyantra.providers.typesafe import typesafe_interpreter

            settings["interpreter"] = await stack.enter_async_context(
                typesafe_interpreter(model=args.jev_model)
            )
        runtime = await stack.enter_async_context(
            RoverRuntime(
                controller=controller,
                assistant=AssistantBridge(**settings),
                allow_agent_motion=args.allow_agent_motion,
                artifacts=args.artifacts,
            )
        )
        await stack.enter_async_context(LocalServer(runtime, args.socket))
        print(
            json.dumps(
                {
                    "event": "ready",
                    "socket": str(args.socket),
                    "simulated_controller": args.simulated,
                    "live_openai": args.live,
                    "interpreter": args.interpret,
                }
            ),
            flush=True,
        )
        await runtime.shutdown.wait()


async def demo():
    async with RoverRuntime() as runtime:

        async def command(op, **arguments):
            return await runtime.dispatch(
                Request(uuid4().hex, op, "operator", arguments)
            )

        await command("arm")
        job = await command("move", distance_m=0.02)
        await runtime.active[1]
        inspection = await command("inspect", describe=True)
        answer = await command("ask", text="What do you see?")
        await command("bumper", pressed=True)
        return {
            "mode": "offline_simulation",
            "motion_job": runtime.jobs[job["id"]],
            "inspection": inspection,
            "assistant": answer,
            "fault_example": runtime.status(),
            "limitations": (
                "Ideal kinematics and fixture inference; no hardware validation"
            ),
        }


async def run(args):
    if args.command == "session":
        await serve(args)
        return 0
    if args.command == "demo":
        output = record(await demo())
        success = output["motion_job"]["state"] == "completed"
    else:
        output = await send(args.socket, request_for(args))
        success = output["ok"]
    print(json.dumps(output, allow_nan=False, indent=None if args.json else 2))
    return 0 if success else 1


def main():
    from pragyantra.cli.environment import cli_environment

    try:
        with cli_environment(sys.argv[1:], prog="prag-rover") as argv:
            args = parser().parse_args(argv)
            code = asyncio.run(run(args))
    except KeyboardInterrupt:
        code = 130
    except (ValueError, OSError, TimeoutError, RuntimeError) as exc:
        print(json.dumps({"ok": False, "error": str(exc)}), file=sys.stderr)
        code = 1
    raise SystemExit(code)
