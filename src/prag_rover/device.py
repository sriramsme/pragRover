"""Async client for the motion firmware's authenticated local-LAN TCP protocol.

The shared token pairs a trusted LAN host; TCP is not encrypted. Tailscale/SSH
protect remote host access, not this separate robot link. No reconnect or replay.
"""

import asyncio
import json
import math
import re
import time
from datetime import UTC, datetime

from prag_rover.controller import Geometry, State
from prag_rover.protocol import _reject_constant, _unique_object, encode, number


class DeviceRejected(ValueError):
    """Authenticated firmware rejected a request without executing it."""


class DeviceController:
    simulated = False
    MAX_LEASE = 0.3

    def __init__(
        self,
        host,
        port,
        token,
        *,
        timeout=0.2,
        connect=asyncio.open_connection,
        now=time.monotonic,
    ):
        if not isinstance(token, str) or not 16 <= len(token) <= 128:
            raise ValueError("Controller token must contain 16–128 characters")
        if not host or type(port) is not int or not 1 <= port <= 65535:
            raise ValueError("Controller requires a host and valid port")
        number(timeout, name="controller timeout", minimum=0.01, maximum=1)
        self.host, self.port, self._token = host, port, token
        self.timeout, self._connect, self.now = timeout, connect, now
        self.reader = self.writer = None
        self.connected = False
        self.session = self.ticket = None
        self.sequence = 0
        self._ticket_at = self._received_at = 0.0
        self._ticket_ttl = 0.0
        self._telemetry = None
        self.geometry = None
        self._lock = asyncio.Lock()

    @property
    def state(self):
        return State(self._telemetry["state"]) if self._telemetry else State.DISARMED

    @property
    def fault(self):
        return self._telemetry["fault"] if self._telemetry else None

    @property
    def left_distance(self):
        return self._telemetry["left_distance_m"]

    @property
    def right_distance(self):
        return self._telemetry["right_distance_m"]

    async def start(self):
        try:
            async with asyncio.timeout(self.timeout):
                self.reader, self.writer = await self._connect(
                    self.host,
                    self.port,
                    limit=16384,
                )
                await self._exchange(
                    {"version": 1, "op": "hello", "token": self._token}, hello=True
                )
            self.connected = True
        except BaseException:
            await self.close()
            raise

    async def close(self):
        self.connected = False
        if self.writer is not None:
            writer, self.writer = self.writer, None
            writer.close()
            try:
                await writer.wait_closed()
            except OSError:
                pass

    def _accept(self, response, *, hello):
        if (
            not isinstance(response, dict)
            or type(response.get("version")) is not int
            or response["version"] != 1
            or type(response.get("ok")) is not bool
        ):
            raise RuntimeError("Invalid firmware response envelope")
        # Auth failures may omit all session information; never expose secrets.
        if hello and not response["ok"]:
            raise DeviceRejected("Controller rejected pairing or configuration")
        for name in ("session", "ticket"):
            if not isinstance(response.get(name), str) or not re.fullmatch(
                r"[0-9a-f]{8,64}",
                response[name],
            ):
                raise RuntimeError(f"Invalid firmware {name}")
        if not hello and response["session"] != self.session:
            raise RuntimeError(
                "Controller session changed; explicit reconnection required"
            )
        ttl = number(
            response.get("ticket_ttl_ms"), name="ticket TTL", minimum=1, maximum=1000
        )
        telemetry = response.get("telemetry")
        if not isinstance(telemetry, dict):
            raise RuntimeError("Firmware telemetry is missing")
        State(telemetry.get("state"))
        if (
            telemetry.get("simulated") is not False
            or type(telemetry.get("bumper")) is not bool
        ):
            raise RuntimeError("Invalid firmware provenance or bumper value")
        if telemetry.get("fault") is not None and not isinstance(
            telemetry["fault"], str
        ):
            raise RuntimeError("Invalid firmware fault")
        for key in (
            "monotonic_us",
            "last_motion_sequence",
            "left_counts",
            "right_counts",
        ):
            if type(telemetry.get(key)) is not int:
                raise RuntimeError(f"Invalid firmware integer: {key}")
        if telemetry["monotonic_us"] < 0 or telemetry["last_motion_sequence"] < -1:
            raise RuntimeError("Invalid firmware timestamp/sequence")
        if (
            self._telemetry is not None
            and telemetry["monotonic_us"] < self._telemetry["monotonic_us"]
        ):
            raise RuntimeError("Controller clock moved backwards")
        for key in (
            "left_mps",
            "right_mps",
            "left_distance_m",
            "right_distance_m",
            "x_m",
            "y_m",
            "heading_rad",
        ):
            value = telemetry.get(key)
            if type(value) not in (int, float) or not math.isfinite(value):
                raise RuntimeError(f"Invalid firmware measurement: {key}")
        # Unconfigured boardless firmware may report zero calibration. It can be
        # observed but cannot run motion jobs until firmware and host validate it.
        geometry = None
        if all(
            telemetry.get(key, 0) > 0
            for key in (
                "wheel_radius_m",
                "track_m",
                "counts_per_revolution",
            )
        ):
            geometry = Geometry(
                telemetry["wheel_radius_m"],
                telemetry["track_m"],
                telemetry["counts_per_revolution"],
            )
        self.geometry = geometry
        self.session, self.ticket = response["session"], response["ticket"]
        self._telemetry = telemetry
        self._received_at = self._ticket_at = self.now()
        self._received_utc = datetime.now(UTC)
        self._ticket_ttl = ttl / 1000
        if not response["ok"]:
            raise DeviceRejected(
                "Controller rejected the command: "
                + str(response.get("error", "unknown"))[:160]
            )

    async def _exchange(self, request, *, hello=False):
        if self.writer is None:
            raise RuntimeError("Controller is disconnected; no automatic replay")
        self.writer.write(encode(request))
        await self.writer.drain()
        line = await self.reader.readline()
        if not line or len(line) > 16384:
            raise RuntimeError("Controller closed or exceeded its response limit")
        try:
            response = json.loads(
                line, object_pairs_hook=_unique_object, parse_constant=_reject_constant
            )
            self._accept(response, hello=hello)
        except DeviceRejected:
            raise
        except (ValueError, TypeError, KeyError) as exc:
            raise RuntimeError("Malformed firmware response") from exc

    async def _request(self, operation, *, expires_at=None, **arguments):
        try:
            async with asyncio.timeout(self.timeout):
                async with self._lock:
                    if not self.connected:
                        raise RuntimeError(
                            "Controller is disconnected; restart the session"
                        )
                    if operation not in ("status", "stop") and (
                        self.now() - self._ticket_at >= self._ticket_ttl / 2
                    ):
                        await self._send_command("status")
                    if expires_at is not None:
                        # Preserve the caller's deadline across lock contention
                        # and ticket refresh; queued work earns no extra lease.
                        lease = number(
                            expires_at - self.now(),
                            name="remaining lease",
                            minimum=0.001,
                            maximum=self.MAX_LEASE,
                        )
                        arguments["lease_ms"] = int(lease * 1000)
                    await self._send_command(operation, **arguments)
        except DeviceRejected:
            raise
        except BaseException:
            # A cancelled/timed-out stream may contain an unknown response.
            # Close it rather than mismatching or replaying buffered commands.
            await self.close()
            raise

    async def _send_command(self, operation, **arguments):
        self.sequence += 1
        await self._exchange(
            {
                "version": 1,
                "op": operation,
                "session": self.session,
                "ticket": self.ticket,
                "seq": self.sequence,
                **arguments,
            }
        )

    async def tick(self):
        await self._request("status")

    async def arm(self):
        await self._request("arm")
        if self.state != State.READY or self.geometry is None:
            raise RuntimeError("Controller did not confirm a calibrated ready state")

    async def reset(self):
        await self._request("reset")

    async def drive(self, left, right, *, sequence, expires_at):
        left = number(left, name="left speed", minimum=-0.15, maximum=0.15)
        right = number(right, name="right speed", minimum=-0.15, maximum=0.15)
        number(
            expires_at - self.now(), name="lease", minimum=0.001, maximum=self.MAX_LEASE
        )
        await self._request(
            "drive",
            left_mps=left,
            right_mps=right,
            expires_at=expires_at,
        )

    async def finish_segment(self):
        await self._request("drive", left_mps=0, right_mps=0, lease_ms=100)

    async def stop(self, fault=None):
        await self._request("stop")

    def snapshot(self):
        if self._telemetry is None:
            raise RuntimeError("No controller telemetry has been acquired")
        return {
            **self._telemetry,
            "connected": self.connected,
            "host_received_age_s": self.now() - self._received_at,
            "host_received_at": self._received_utc.isoformat(),
        }
