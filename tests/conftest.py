"""Normal prototype tests cannot call hosted services or open IP connections."""

import socket

import pytest


@pytest.fixture(autouse=True)
def offline(monkeypatch):
    for key in ("OPENAI_API_KEY", "TYPESAFE_API_KEY"):
        monkeypatch.delenv(key, raising=False)
    original_connect = socket.socket.connect
    original_connect_ex = socket.socket.connect_ex

    def connect(self, address):
        if self.family != socket.AF_UNIX:
            raise AssertionError("Tests must not open IP connections")
        return original_connect(self, address)

    def connect_ex(self, address):
        if self.family != socket.AF_UNIX:
            raise AssertionError("Tests must not open IP connections")
        return original_connect_ex(self, address)

    monkeypatch.setattr(socket.socket, "connect", connect)
    monkeypatch.setattr(socket.socket, "connect_ex", connect_ex)
