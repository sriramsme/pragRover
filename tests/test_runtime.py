import asyncio
from uuid import uuid4

import pytest
from pragyantra.application.requests import RequestStatus

from prag_rover.assistant import AssistantBridge
from prag_rover.controller import State
from prag_rover.protocol import Request, decode_request
from prag_rover.runtime import RoverRuntime


async def command(runtime, operation, *, owner="operator", **arguments):
    return await runtime.dispatch(Request(uuid4().hex, operation, owner, arguments))


def test_motion_completion_and_real_pragyantra_inspection():
    async def scenario():
        async with RoverRuntime() as runtime:
            await command(runtime, "arm")
            job = await command(runtime, "move", distance_m=0.01)
            await runtime.active[1]
            assert runtime.jobs[job["id"]]["state"] == "completed"
            assert 0.009 <= runtime.controller.x <= 0.02
            assert runtime.controller.state == State.DISARMED
            inspection = await command(runtime, "inspect", describe=True)
            assert inspection["scene"].provider == "fixture"
            assert inspection["judgment"]["batch"]["judgments"][0]["refused"]
            answer = await command(runtime, "ask", text="What do you see?")
            assert answer.status == RequestStatus.INSPECT
            assert answer.judgment.provider == "fixture"
            assert "Synthetic" in answer.text

    asyncio.run(scenario())


def test_immediate_cancel_bumper_latching_and_agent_policy():
    async def scenario():
        async with RoverRuntime() as runtime:
            with pytest.raises(ValueError):
                await command(runtime, "arm", owner="agent:muse")
            await command(runtime, "arm")
            job = await command(runtime, "mission", name="desk-demo")
            await command(runtime, "stop", owner="agent:dots")
            assert runtime.jobs[job["id"]]["state"] == "cancelled"
            assert runtime.controller.left == 0
            await command(runtime, "bumper", pressed=True)
            with pytest.raises(ValueError):
                await command(runtime, "reset")
            await command(runtime, "bumper", pressed=False)
            assert runtime.controller.state == State.FAULT
            await command(runtime, "reset")
            assert runtime.controller.state == State.DISARMED
        async with RoverRuntime(allow_agent_motion=True) as runtime:
            await command(runtime, "arm", owner="agent:muse")
            with pytest.raises(ValueError):
                await command(
                    runtime, "drive", owner="agent:muse", left_mps=0.1, right_mps=0.1
                )
            with pytest.raises(ValueError):
                await command(runtime, "mission", owner="agent:dots", name="look")
            job = await command(runtime, "mission", owner="agent:muse", name="look")
            await runtime.active[1]
            assert runtime.jobs[job["id"]]["state"] == "completed"

    asyncio.run(scenario())


def test_stop_bypasses_blocked_conversation_and_prevents_resumed_motion():
    class SlowConversation:
        def __init__(self):
            self.started = asyncio.Event()
            self.release = asyncio.Event()

        async def respond(self, request):
            self.started.set()
            await self.release.wait()
            from pragyantra.application.conversation import GeneratedReply

            return GeneratedReply("Done", "test", "slow")

    async def scenario():
        provider = SlowConversation()
        assistant = AssistantBridge(conversation=provider)
        async with RoverRuntime(assistant=assistant) as runtime:
            await command(runtime, "arm")
            job = await command(runtime, "move", distance_m=1)
            chat = asyncio.create_task(command(runtime, "ask", text="hello"))
            await provider.started.wait()
            await command(runtime, "ask", text="/stop")
            assert runtime.jobs[job["id"]]["state"] == "cancelled"
            assert runtime.controller.left == runtime.controller.right == 0
            provider.release.set()
            await chat
            assert runtime.controller.state == State.DISARMED

    asyncio.run(scenario())


def test_protocol_rejects_duplicates_unknown_fields_and_versions():
    for payload in (
        b'{"version":1,"version":1}',
        b'{"version":true,"id":"a","operation":"stop","owner":"operator","arguments":{}}',
        b'{"version":1,"id":"a","operation":"stop","owner":"operator","arguments":{},"extra":1}',
        b"x" * 16385,
    ):
        with pytest.raises(ValueError):
            decode_request(payload)
