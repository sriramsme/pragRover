"""Exercise the real host transport against a fake firmware peer over Unix IPC."""

import asyncio
import json

import pytest

from prag_rover.controller import State
from prag_rover.device import DeviceController, DeviceRejected
from prag_rover.runtime import RoverRuntime


class FirmwarePeer:
    def __init__(self):
        self.requests = []
        self.state = "disarmed"
        self.ticket = 0
        self.clock = 0
        self.left = self.right = 0
        self.position = 0
        self.malformed = False
        self.reject = False
        self.hold = False
        self.started = asyncio.Event()
        self.release = asyncio.Event()
        self.counts_per_revolution = 20

    async def handle(self, reader, writer):
        try:
            while line := await reader.readline():
                request = json.loads(line)
                self.requests.append(request)
                if self.hold:
                    self.started.set()
                    await self.release.wait()
                op = request["op"]
                if op == "hello":
                    assert request["token"] == "test-token-123456789"
                else:
                    assert request["session"] == "a" * 16
                    assert request["ticket"] == f"{self.ticket:016x}"
                if not self.reject:
                    if op == "arm":
                        self.state = "ready"
                    elif op == "drive":
                        self.left, self.right = (
                            request["left_mps"],
                            request["right_mps"],
                        )
                        self.state = "moving" if self.left or self.right else "ready"
                    elif op in ("stop", "reset", "hello"):
                        self.state = "disarmed"
                        self.left = self.right = 0
                self.clock += 20000
                self.position += self.left * 0.02
                self.ticket += 1
                response = {
                    "version": 1,
                    "ok": not self.reject,
                    "session": "a" * 16,
                    "ticket": f"{self.ticket:016x}",
                    "ticket_ttl_ms": 250,
                    "error": "wiring_unconfirmed" if self.reject else None,
                    "telemetry": {
                        "simulated": False,
                        "state": self.state,
                        "fault": None,
                        "bumper": False,
                        "monotonic_us": self.clock,
                        "left_mps": self.left,
                        "right_mps": self.right,
                        "left_distance_m": self.position,
                        "right_distance_m": self.position,
                        "left_counts": 0,
                        "right_counts": 0,
                        "x_m": self.position,
                        "y_m": 0,
                        "heading_rad": 0,
                        "last_motion_sequence": request.get("seq", -1),
                        "wheel_radius_m": 0.0325,
                        "track_m": 0.14,
                        "counts_per_revolution": self.counts_per_revolution,
                    },
                }
                if self.malformed:
                    response["telemetry"]["bumper"] = 1
                writer.write((json.dumps(response) + "\n").encode())
                await writer.drain()
        finally:
            writer.close()
            await writer.wait_closed()


async def connect_peer(tmp_path, peer):
    path = tmp_path / "device.sock"
    server = await asyncio.start_unix_server(peer.handle, path)

    async def connect(host, port, *, limit):
        return await asyncio.open_unix_connection(path, limit=limit)

    device = DeviceController(
        "test-host", 8765, "test-token-123456789", connect=connect
    )
    return server, device


def test_ticket_handshake_commands_and_telemetry(tmp_path):
    async def scenario():
        peer = FirmwarePeer()
        server, device = await connect_peer(tmp_path, peer)
        async with server:
            await device.start()
            assert device.state == State.DISARMED
            assert device.snapshot()["simulated"] is False
            assert device.snapshot()["last_motion_sequence"] == -1
            assert device.geometry.track_m == 0.14
            await device.arm()
            await device.drive(0.1, 0.1, sequence=1, expires_at=device.now() + 0.299)
            assert device.state == State.MOVING
            await device.finish_segment()
            assert device.state == State.READY
            await device.stop()
            assert device.state == State.DISARMED
            requests = peer.requests[1:]
            assert [r["seq"] for r in requests] == list(range(1, len(requests) + 1))
            assert len({r["ticket"] for r in requests}) == len(requests)
            await device.close()
            assert not device.connected

    asyncio.run(scenario())


def test_rejection_preserves_channel_but_malformed_response_closes(tmp_path):
    async def scenario():
        peer = FirmwarePeer()
        server, device = await connect_peer(tmp_path, peer)
        async with server:
            await device.start()
            peer.reject = True
            with pytest.raises(DeviceRejected, match="wiring_unconfirmed"):
                await device.arm()
            assert device.connected
            peer.reject = False
            peer.malformed = True
            with pytest.raises(RuntimeError, match="bumper"):
                await device.tick()
            assert not device.connected

    asyncio.run(scenario())


def test_expired_host_ticket_is_refreshed_before_arm(tmp_path):
    async def scenario():
        peer = FirmwarePeer()
        server, device = await connect_peer(tmp_path, peer)
        async with server:
            await device.start()
            device._ticket_at -= 1  # Age the local ticket without a wall-clock sleep.
            await device.arm()
            assert [r["op"] for r in peer.requests] == ["hello", "status", "arm"]
            await device.close()

    asyncio.run(scenario())


def test_cancelled_request_closes_without_replay(tmp_path):
    async def scenario():
        peer = FirmwarePeer()
        server, device = await connect_peer(tmp_path, peer)
        async with server:
            await device.start()
            peer.hold = True
            request = asyncio.create_task(device.arm())
            await peer.started.wait()
            request.cancel()
            with pytest.raises(asyncio.CancelledError):
                await request
            assert not device.connected
            peer.hold = False
            peer.release.set()
            with pytest.raises(RuntimeError, match="disconnected"):
                await device.arm()
            assert [r["op"] for r in peer.requests] == ["hello", "arm"]

    asyncio.run(scenario())


def test_runtime_can_use_async_controller_and_reject_simulated_injection(tmp_path):
    from prag_rover.protocol import Request

    async def scenario():
        peer = FirmwarePeer()
        server, device = await connect_peer(tmp_path, peer)
        async with server, RoverRuntime(controller=device) as runtime:
            await runtime.dispatch(Request("arm", "arm", "operator", {}))
            assert runtime.owner == "operator"
            with pytest.raises(ValueError, match="simulation"):
                await runtime.dispatch(
                    Request("bumper", "bumper", "operator", {"pressed": True})
                )
            await runtime.dispatch(Request("stop", "stop", "operator", {}))
            assert runtime.owner is None
            assert device.state == State.DISARMED
        assert not device.connected

    asyncio.run(scenario())


def test_motion_uses_received_odometry_and_stops_on_completion(tmp_path):
    from prag_rover.protocol import Request

    async def scenario():
        peer = FirmwarePeer()
        server, device = await connect_peer(tmp_path, peer)

        async def yield_only(delay):
            await asyncio.sleep(0)

        async with server, RoverRuntime(controller=device, sleep=yield_only) as runtime:
            await runtime.dispatch(Request("arm", "arm", "operator", {}))
            job = await runtime.dispatch(
                Request("move", "move", "operator", {"distance_m": 0.01})
            )
            async with asyncio.timeout(2):
                await runtime.active[1]
            assert runtime.jobs[job["id"]]["state"] == "completed"
            assert runtime.jobs[job["id"]]["result"]["simulated"] is False
            assert device.state == State.DISARMED

    asyncio.run(scenario())


@pytest.mark.parametrize("queue_delay", [0.08, 0.11])
def test_queued_drive_preserves_original_host_deadline(tmp_path, queue_delay):
    async def scenario():
        peer = FirmwarePeer()
        peer.counts_per_revolution = 20.5
        server, device = await connect_peer(tmp_path, peer)
        clock = [0.0]
        device.now = lambda: clock[0]
        async with server:
            await device.start()
            assert device.geometry.counts_per_revolution == 20.5
            await device.arm()
            async with device._lock:
                pending = asyncio.create_task(
                    device.drive(0.1, 0.1, sequence=1, expires_at=0.1)
                )
                await asyncio.sleep(0)
                clock[0] += queue_delay
            if queue_delay < 0.1:
                await pending
                command = peer.requests[-1]
                assert command["op"] == "drive"
                assert 1 <= command["lease_ms"] <= 20
            else:
                with pytest.raises(ValueError, match="remaining lease"):
                    await pending
                assert all(r["op"] != "drive" for r in peer.requests)
                assert not device.connected
            await device.close()

    asyncio.run(scenario())


def test_ticket_refresh_consumes_existing_drive_lease(tmp_path):
    async def scenario():
        peer = FirmwarePeer()
        server, device = await connect_peer(tmp_path, peer)
        clock = [0.0]
        device.now = lambda: clock[0]
        async with server:
            await device.start()
            await device.arm()
            send_command = device._send_command

            async def delayed_refresh(operation, **arguments):
                await send_command(operation, **arguments)
                if operation == "status":
                    clock[0] += 0.08

            device._send_command = delayed_refresh
            clock[0] = 0.13  # Refresh needed before this drive can be submitted.
            await device.drive(0.1, 0.1, sequence=1, expires_at=0.38)
            assert [r["op"] for r in peer.requests] == [
                "hello",
                "arm",
                "status",
                "drive",
            ]
            assert 160 <= peer.requests[-1]["lease_ms"] <= 170
            await device.close()

    asyncio.run(scenario())
