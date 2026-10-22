import asyncio
from uuid import uuid4

import pytest
from pragyantra.application.conversation import GeneratedReply

from prag_rover.assistant import AssistantBridge
from prag_rover.controller import State
from prag_rover.ipc import LocalServer, send
from prag_rover.protocol import Request
from prag_rover.runtime import RoverRuntime


def request(operation, **arguments):
    return Request(uuid4().hex, operation, "operator", arguments)


def run(scenario):
    async def bounded():
        async with asyncio.timeout(5):
            await scenario()

    asyncio.run(bounded())


def test_stop_waits_for_old_job_cleanup_before_accepting_rearm():
    async def scenario():
        async with RoverRuntime() as runtime:
            entered = asyncio.Event()
            cancelling = asyncio.Event()
            release_cleanup = asyncio.Event()

            async def work():
                entered.set()
                try:
                    await asyncio.Event().wait()
                except asyncio.CancelledError:
                    cancelling.set()
                    await release_cleanup.wait()
                    raise

            await runtime.dispatch(request("arm"))
            job = runtime._start_job("operator", "cleanup-test", work)
            await entered.wait()
            stopping = asyncio.create_task(runtime.dispatch(request("stop")))
            try:
                await cancelling.wait()
                assert runtime.controller.state == State.DISARMED
                assert not stopping.done()
                with pytest.raises(ValueError, match="stop"):
                    await runtime.dispatch(request("arm"))
            finally:
                release_cleanup.set()
                await stopping
            assert runtime.jobs[job["id"]]["state"] == "cancelled"
            await runtime.dispatch(request("arm"))
            assert runtime.owner == "operator"
            assert runtime.controller.state == State.READY

    run(scenario)


def test_competing_start_cannot_replace_socket_during_partial_startup(
    tmp_path, monkeypatch
):
    async def scenario():
        path = tmp_path / "rover.sock"
        bound = asyncio.Event()
        finish_start = asyncio.Event()
        real_start = asyncio.start_unix_server

        async def delayed_start(*args, **kwargs):
            # LocalServer must already own its atomically bound raw socket.
            assert kwargs.get("sock") is not None
            assert "path" not in kwargs
            bound.set()
            await finish_start.wait()
            return await real_start(*args, **kwargs)

        monkeypatch.setattr(asyncio, "start_unix_server", delayed_start)
        async with RoverRuntime() as runtime, RoverRuntime() as other:
            server = LocalServer(runtime, path)
            starting = asyncio.create_task(server.__aenter__())
            try:
                await bound.wait()
                inode = path.lstat().st_ino
                with pytest.raises(ValueError, match="bind"):
                    async with LocalServer(other, path):
                        pass
                assert path.lstat().st_ino == inode
            finally:
                finish_start.set()
                await starting
            try:
                assert (await send(path, request("status")))["ok"]
            finally:
                await server.__aexit__(None, None, None)
            assert not path.exists()

    run(scenario)


@pytest.mark.parametrize("cancel", [False, True])
def test_partial_server_start_releases_owned_socket(tmp_path, monkeypatch, cancel):
    async def failed_start(*args, **kwargs):
        if cancel:
            raise asyncio.CancelledError
        raise OSError("injected startup failure")

    monkeypatch.setattr(asyncio, "start_unix_server", failed_start)

    async def scenario():
        path = tmp_path / "rover.sock"
        async with RoverRuntime() as runtime:
            exception = asyncio.CancelledError if cancel else OSError
            with pytest.raises(exception):
                async with LocalServer(runtime, path):
                    pass
            assert not path.exists()

    run(scenario)


def test_stop_ask_bypasses_saturated_assistant_admission(tmp_path):
    class BlockedConversation:
        def __init__(self):
            self.started = asyncio.Event()
            self.release = asyncio.Event()

        async def respond(self, request):
            self.started.set()
            await self.release.wait()
            return GeneratedReply("Finished", "test", "blocked")

    async def scenario():
        path = tmp_path / "rover.sock"
        provider = BlockedConversation()
        bridge = AssistantBridge(conversation=provider)
        async with RoverRuntime(assistant=bridge) as runtime:
            async with LocalServer(runtime, path) as server:
                await runtime.dispatch(request("arm"))
                calls = [
                    asyncio.create_task(send(path, request("ask", text="hello")))
                    for _ in range(4)
                ]
                try:
                    async with asyncio.timeout(5):
                        await provider.started.wait()
                        while not server._slow.locked():
                            await asyncio.sleep(0)
                        response = await send(path, request("ask", text="/stop"))
                    assert response["ok"]
                    assert response["result"]["controller"]["state"] == "disarmed"
                    assert all(not call.done() for call in calls)
                finally:
                    provider.release.set()
                    await asyncio.gather(*calls, return_exceptions=True)

    run(scenario)
