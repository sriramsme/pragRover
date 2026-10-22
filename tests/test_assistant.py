import asyncio
from contextlib import contextmanager
from datetime import UTC, datetime, timedelta
from io import BytesIO

from PIL import Image
from pragyantra.application.conversation import GeneratedReply
from pragyantra.application.inspection import ImageJudgment, JudgmentBatch
from pragyantra.application.models import SceneDescription
from pragyantra.application.requests import Intent, IntentJudgment

from prag_rover.assistant import AssistantBridge, HttpSnapshotCamera


def test_configured_provider_boundaries_are_used_without_fixture_fallback():
    class Providers:
        def __init__(self):
            self.calls = []

        async def respond(self, request):
            self.calls.append("conversation")
            return GeneratedReply("provider reply", "fake-openai", "test")

        async def describe(self, frame, *, previous=None):
            self.calls.append("vision")
            return SceneDescription("A test target", "fake-openai", "test")

        async def evaluate(self, frame, questions):
            self.calls.append("decisions")
            return JudgmentBatch(
                tuple(ImageJudgment(q, probability=0.8) for q in questions),
                "fake-openai-decisions",
                "test",
            )

        async def interpret(self, message):
            self.calls.append("jev")
            return IntentJudgment(
                Intent.CONVERSATION,
                tuple((i, float(i == Intent.CONVERSATION)) for i in Intent),
                1,
                "fake-jev",
                "test",
            )

    async def scenario():
        provider = Providers()
        bridge = AssistantBridge(
            conversation=provider,
            vision=provider,
            inspection=provider,
            interpreter=provider,
        )
        inspected = await bridge.inspect("Is the target visible?", describe=True)
        answer = await bridge.ask("hello")
        assert inspected["judgment"]["batch"]["provider"] == "fake-openai-decisions"
        assert answer.conversation.reply.text == "provider reply"
        assert provider.calls == ["decisions", "vision", "jev", "conversation"]

    asyncio.run(scenario())


def test_http_snapshot_normalization_and_timestamp_meaning(monkeypatch):
    data = BytesIO()
    Image.new("RGB", (1200, 800), "red").save(data, format="JPEG")

    @contextmanager
    def response(url, timeout):
        assert timeout == 2
        yield BytesIO(data.getvalue())

    monkeypatch.setattr("prag_rover.assistant.urlopen", response)
    before = datetime.now(UTC)
    frame = asyncio.run(HttpSnapshotCamera("http://camera/capture").read())
    assert before <= frame.observed_at <= datetime.now(UTC)
    assert frame.source == "rover.http-read-start"
    with Image.open(BytesIO(frame.data)) as image:
        assert image.size == (1024, 683)


def test_expired_inspection_evidence_is_not_treated_as_current():
    from pragyantra.application.models import ImageFrame

    class OldCamera:
        async def read(self):
            return ImageFrame(
                "old", datetime.now(UTC) - timedelta(seconds=20), b"fixture"
            )

    result = asyncio.run(
        AssistantBridge(camera=OldCamera()).inspect("Is anything visible?")
    )
    assert result["judgment"]["status"] == "stale"


def test_direct_decision_does_not_invoke_generative_vision():
    class ForbiddenVision:
        async def describe(self, frame, *, previous=None):
            raise AssertionError("Direct decisions must not require a caption")

    result = asyncio.run(
        AssistantBridge(vision=ForbiddenVision()).inspect("Is there a target?")
    )
    assert result["scene"] is None
    assert result["judgment"]["batch"]["judgments"][0]["refused"]


def test_camera_bearer_token_is_header_only_and_redirects_are_refused(monkeypatch):
    from prag_rover.assistant import _NoRedirect

    data = BytesIO()
    Image.new("RGB", (64, 64), "green").save(data, format="JPEG")

    class Opener:
        def open(self, request, *, timeout):
            assert request.full_url == "http://camera/capture"
            assert (
                request.get_header("Authorization")
                == "Bearer private-camera-token-1234"
            )
            assert timeout == 2
            return BytesIO(data.getvalue())

    def opener(handler):
        assert isinstance(handler, _NoRedirect)
        return Opener()

    monkeypatch.setattr("prag_rover.assistant.build_opener", opener)
    frame = asyncio.run(
        HttpSnapshotCamera(
            "http://camera/capture", token="private-camera-token-1234"
        ).read()
    )
    assert "private-camera-token-1234" not in frame.source
    import pytest

    with pytest.raises(ValueError, match="redirects"):
        _NoRedirect().redirect_request(None, None, 302, "", {}, "http://other-camera/")


def test_physical_sensor_context_uses_host_receipt_time_not_ask_time():
    bridge = AssistantBridge()
    received = datetime.now(UTC) - timedelta(seconds=2)
    bridge.update_telemetry(
        {
            "simulated": False,
            "host_received_at": received.isoformat(),
            "left_mps": 0,
            "right_mps": 0,
            "x_m": 1,
            "y_m": 0,
            "heading_rad": 0,
            "bumper": False,
        }
    )
    snapshot = bridge.session.state.snapshot(at=datetime.now(UTC))
    assert all(s.observed_at == received for s in snapshot.sensors)
    assert all(s.source == "rover.controller-receipt" for s in snapshot.sensors)
