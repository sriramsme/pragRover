"""Version-1 local host requests, not an ESP32 wire protocol or authorization."""

import json
import math
import re
from dataclasses import dataclass

MAX_RECORD = 16384
OPERATIONS = {
    "status",
    "arm",
    "disarm",
    "stop",
    "reset",
    "takeover",
    "drive",
    "move",
    "turn",
    "mission",
    "jobs",
    "bumper",
    "link",
    "ask",
    "inspect",
    "capture",
    "shutdown",
}


def number(value, *, name, minimum, maximum):
    if (
        type(value) not in (int, float)
        or not math.isfinite(value)
        or not minimum <= value <= maximum
    ):
        raise ValueError(f"{name} must be finite and within [{minimum}, {maximum}]")
    return float(value)


def fields(values, required=(), optional=()):
    if not isinstance(values, dict) or not set(required) <= values.keys():
        raise ValueError("Missing command arguments")
    if values.keys() - set(required) - set(optional):
        raise ValueError("Unexpected command arguments")


@dataclass(frozen=True)
class Request:
    id: str
    operation: str
    owner: str
    arguments: dict

    def __post_init__(self):
        if not isinstance(self.id, str) or not re.fullmatch(r"[\w-]{1,64}", self.id):
            raise ValueError("Request id requires 1–64 identifier characters")
        if not isinstance(self.operation, str) or self.operation not in OPERATIONS:
            raise ValueError("Unknown operation")
        if not isinstance(self.owner, str) or not re.fullmatch(
            r"operator|agent:[a-zA-Z0-9_-]{1,32}", self.owner
        ):
            raise ValueError("Owner must be operator or agent:NAME")
        if not isinstance(self.arguments, dict):
            raise ValueError("Arguments must be an object")

    def record(self):
        return {
            "version": 1,
            "id": self.id,
            "operation": self.operation,
            "owner": self.owner,
            "arguments": self.arguments,
        }


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("Duplicate JSON key")
        result[key] = value
    return result


def _reject_constant(value):
    raise ValueError(f"Non-JSON numeric constant: {value}")


def decode_request(line: bytes) -> Request:
    if len(line) > MAX_RECORD:
        raise ValueError("Request exceeds 16 KiB")
    try:
        data = json.loads(
            line,
            object_pairs_hook=_unique_object,
            parse_constant=_reject_constant,
        )
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError("Request must be UTF-8 JSON") from exc
    fields(data, ("version", "id", "operation", "owner", "arguments"))
    if type(data["version"]) is not int or data["version"] != 1:
        raise ValueError("Only integer protocol version 1 is supported")
    return Request(data["id"], data["operation"], data["owner"], data["arguments"])


def encode(record):
    return (json.dumps(record, allow_nan=False, separators=(",", ":")) + "\n").encode()
