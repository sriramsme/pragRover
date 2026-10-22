import asyncio
import json
import os
import sys
from uuid import uuid4

import pytest

from prag_rover.ipc import LocalServer, send
from prag_rover.protocol import Request
from prag_rover.runtime import RoverRuntime


def request(operation, **arguments):
    return Request(uuid4().hex, operation, "operator", arguments)


def test_cli_defaults_store_runtime_data_under_working_directory(tmp_path, monkeypatch):
    from prag_rover.cli import parser

    monkeypatch.chdir(tmp_path)
    arguments = parser().parse_args(["session", "--simulated"])
    assert arguments.socket.resolve() == tmp_path / "data" / "rover.sock"
    assert arguments.artifacts.resolve() == tmp_path / "data" / "captures"


def test_socket_ownership_deduplication_and_stop(tmp_path):
    async def scenario():
        path = tmp_path / "rover.sock"
        async with RoverRuntime() as runtime, LocalServer(runtime, path):
            assert path.stat().st_mode & 0o777 == 0o600
            async with RoverRuntime() as other:
                with pytest.raises(ValueError):
                    async with LocalServer(other, path):
                        pass
            assert (await send(path, request("arm")))["ok"]
            motion = request("move", distance_m=1)
            first = await send(path, motion)
            second = await send(path, motion)
            assert first == second
            assert len(runtime.jobs) == 1
            conflict = Request(motion.id, "turn", "operator", {"angle_deg": 10})
            assert not (await send(path, conflict))["ok"]
            assert (await send(path, request("stop")))["ok"]
            job = runtime.jobs[first["result"]["id"]]
            assert job["state"] == "cancelled"
            status = await send(path, request("status"))
            assert status["result"]["controller"]["left_mps"] == 0
        assert not path.exists()

    asyncio.run(scenario())


def test_real_cli_session_mission_chat_capture_and_shutdown(tmp_path):
    async def scenario():
        path = tmp_path / "rover.sock"
        captures = tmp_path / "captures"
        env = {
            key: value
            for key, value in os.environ.items()
            if key not in ("OPENAI_API_KEY", "TYPESAFE_API_KEY")
        }
        process = await asyncio.create_subprocess_exec(
            sys.executable,
            "-m",
            "prag_rover",
            "--no-env",
            "--socket",
            str(path),
            "session",
            "--simulated",
            "--artifacts",
            str(captures),
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
            env=env,
        )
        try:
            async with asyncio.timeout(10):
                startup = await process.stdout.readline()
                assert json.loads(startup)["event"] == "ready"

                async def cli(*arguments):
                    child = await asyncio.create_subprocess_exec(
                        sys.executable,
                        "-m",
                        "prag_rover",
                        "--no-env",
                        "--json",
                        "--socket",
                        str(path),
                        *arguments,
                        stdout=asyncio.subprocess.PIPE,
                        stderr=asyncio.subprocess.PIPE,
                        env=env,
                    )
                    out, err = await child.communicate()
                    assert child.returncode == 0, err.decode() + out.decode()
                    return json.loads(out)

                await cli("arm")
                await cli("mission", "look")
                # The next inspection waits for the mission if still active;
                # await its state via the server's short job operation.
                for _ in range(20):
                    jobs = await cli("jobs")
                    if jobs["result"][0]["state"] != "running":
                        break
                assert jobs["result"][0]["state"] == "completed"
                answer = await cli("ask", "What do you see?")
                assert answer["result"]["status"] == "inspect"
                captured = await cli("capture")
                from pathlib import Path

                assert Path(captured["result"]["path"]).is_file()
                await cli("shutdown")
                await process.wait()
                assert process.returncode == 0
                assert not path.exists()
        finally:
            if process.returncode is None:
                process.terminate()
                await process.wait()

    asyncio.run(scenario())
