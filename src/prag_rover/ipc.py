"""Private local IPC for CLI/SSH clients. Not a TCP or physical-device server."""

import asyncio
import dataclasses
import json
import socket
from collections import OrderedDict
from datetime import datetime, timedelta
from enum import Enum
from pathlib import Path

from prag_rover.protocol import MAX_RECORD, Request, decode_request, encode


def record(value):
    if dataclasses.is_dataclass(value):
        return record(dataclasses.asdict(value))
    if isinstance(value, dict):
        return {key: record(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [record(item) for item in value]
    if isinstance(value, datetime):
        return value.isoformat()
    if isinstance(value, timedelta):
        return value.total_seconds()
    if isinstance(value, Enum):
        return value.value
    return value


class LocalServer:
    def __init__(self, runtime, path):
        self.runtime, self.path = runtime, Path(path)
        self.cache = OrderedDict()
        self.inflight = {}
        self.clients = set()
        self.server = None
        self._inode = None
        self._slow = asyncio.Semaphore(4)

    async def __aenter__(self):
        self.path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        # Bind atomically ourselves: asyncio's path= behavior may unlink an
        # existing socket, allowing two competing starts to replace its owner.
        listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            try:
                listener.bind(str(self.path))
            except OSError as exc:
                raise ValueError(
                    "Cannot bind socket; check its owner or choose another path"
                ) from exc
            self._inode = self.path.lstat().st_ino
            self.path.chmod(0o600)
            self.server = await asyncio.start_unix_server(
                self._handle,
                sock=listener,
                limit=MAX_RECORD + 1,
            )
        except BaseException:
            listener.close()
            self._unlink_owned()
            raise
        return self

    def _unlink_owned(self):
        try:
            if self.path.lstat().st_ino == self._inode:
                self.path.unlink()
        except FileNotFoundError:
            pass

    async def __aexit__(self, *args):
        self.server.close()
        await self.server.wait_closed()
        tasks = list(self.clients)
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        self._unlink_owned()

    async def _execute(self, request):
        key = (request.owner, request.id)
        signature = encode(request.record())
        if key in self.cache:
            previous, response = self.cache[key]
            if previous != signature:
                raise ValueError("Request id was already used with a different command")
            return response
        if key in self.inflight:
            previous, future = self.inflight[key]
            if previous != signature:
                raise ValueError("Request id is in use with a different command")
            return await asyncio.shield(future)
        if len(self.inflight) >= 32:
            raise ValueError("Too many active requests")
        future = asyncio.get_running_loop().create_future()
        self.inflight[key] = signature, future
        try:
            result = await self.runtime.dispatch(request)
            response = {
                "version": 1,
                "id": request.id,
                "ok": True,
                "result": record(result),
            }
        except asyncio.CancelledError:
            future.cancel()
            raise
        except Exception as exc:
            response = {
                "version": 1,
                "id": request.id,
                "ok": False,
                "error": {"type": type(exc).__name__, "message": str(exc)},
            }
        finally:
            self.inflight.pop(key, None)
        self.cache[key] = signature, response
        while len(self.cache) > 64:
            self.cache.popitem(last=False)
        future.set_result(response)
        return response

    async def _handle(self, reader, writer):
        task = asyncio.current_task()
        self.clients.add(task)
        try:
            async with asyncio.timeout(45):
                line = await reader.readline()
                request = decode_request(line)
                stop_alias = (
                    request.operation == "ask"
                    and isinstance(request.arguments.get("text"), str)
                    and request.arguments["text"].strip() == "/stop"
                )
                if (
                    request.operation in ("ask", "inspect", "capture")
                    and not stop_alias
                ):
                    if self._slow.locked():
                        raise ValueError(
                            "Assistant is busy; control commands remain available"
                        )
                    async with self._slow:
                        response = await self._execute(request)
                else:
                    response = await self._execute(request)
                writer.write(encode(response))
                await writer.drain()
                if request.operation == "shutdown" and response["ok"]:
                    self.runtime.shutdown.set()
        except (ValueError, TimeoutError, OSError) as exc:
            try:
                writer.write(
                    encode(
                        {
                            "version": 1,
                            "id": None,
                            "ok": False,
                            "error": {"type": type(exc).__name__, "message": str(exc)},
                        }
                    )
                )
                await writer.drain()
            except OSError:
                pass
        finally:
            writer.close()
            try:
                await writer.wait_closed()
            except OSError:
                pass
            finally:
                self.clients.discard(task)


async def send(path, request: Request):
    async with asyncio.timeout(46):
        reader, writer = await asyncio.open_unix_connection(path, limit=1024 * 1024)
        try:
            payload = encode(request.record())
            if len(payload) > MAX_RECORD:
                raise ValueError("Request exceeds 16 KiB")
            writer.write(payload)
            await writer.drain()
            line = await reader.readline()
            if not line:
                raise ValueError("Rover session closed without a response")
            return json.loads(line)
        finally:
            writer.close()
            await writer.wait_closed()
