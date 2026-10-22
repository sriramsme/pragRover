"""Compose existing PragYantra capabilities; providers never command motors."""

import asyncio
from datetime import UTC, datetime
from io import BytesIO
from pathlib import Path
from urllib.parse import urlsplit
from urllib.request import HTTPRedirectHandler, Request, build_opener, urlopen

from PIL import Image, UnidentifiedImageError
from pragyantra.application.conversation import (
    ConversationSession,
    GeneratedReply,
)
from pragyantra.application.inspection import (
    ImageInspector,
    ImageJudgment,
    InspectionQuestion,
    JudgmentBatch,
)
from pragyantra.application.models import (
    SceneDescription,
    SensorObservation,
    TextObservation,
)
from pragyantra.application.perception import perceive_image
from pragyantra.application.requests import Intent, IntentJudgment, RequestRouter
from pragyantra.application.runner import AssistantRunner
from pragyantra.inputs.image import MAX_FILE_BYTES, frame_from_image, load_image


class SyntheticCamera:
    async def read(self):
        return frame_from_image(
            Image.new("RGB", (320, 240), "steelblue"),
            source="rover.synthetic",
            observed_at=datetime.now(UTC),
        )


class FileCamera:
    def __init__(self, path):
        self.path = Path(path)

    async def read(self):
        # Timestamp explicitly means reading the file, not historical capture.
        return await asyncio.to_thread(load_image, self.path)


class _NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        raise ValueError("Authenticated camera redirects are not allowed")


class HttpSnapshotCamera:
    def __init__(self, url, *, token=None):
        parts = urlsplit(url)
        if parts.scheme not in ("http", "https") or not parts.hostname:
            raise ValueError("Camera URL requires http(s) and a host")
        if parts.username or parts.password or parts.fragment:
            raise ValueError("Use a camera URL without credentials or fragments")
        self.url = url
        if token is not None and (
            not isinstance(token, str)
            or not 24 <= len(token) <= 240
            or any(not 33 <= ord(c) <= 126 for c in token)
        ):
            raise ValueError(
                "Camera token must contain 24–240 visible ASCII characters"
            )
        self._token = token

    def _read(self):
        at = datetime.now(UTC)
        # A configured trusted URL only; models cannot choose URLs. Native I/O
        # can delay thread completion. This is one bounded JPEG, not MJPEG.
        if self._token:
            request = Request(
                self.url, headers={"Authorization": "Bearer " + self._token}
            )
            response_context = build_opener(_NoRedirect()).open(request, timeout=2)
        else:
            response_context = urlopen(self.url, timeout=2)
        with response_context as response:
            data = response.read(MAX_FILE_BYTES + 1)
        if len(data) > MAX_FILE_BYTES:
            raise ValueError("Camera response exceeds 10 MiB")
        try:
            with Image.open(BytesIO(data)) as image:
                if image.format not in ("JPEG", "PNG", "WEBP"):
                    raise ValueError("Camera must return JPEG/PNG/WebP snapshots")
                return frame_from_image(
                    image, source="rover.http-read-start", observed_at=at
                )
        except (UnidentifiedImageError, OSError) as exc:
            raise ValueError("Camera response is not a decodable snapshot") from exc

    async def read(self):
        task = asyncio.create_task(asyncio.to_thread(self._read))
        try:
            return await asyncio.shield(task)
        except asyncio.CancelledError:
            # Account for the worker before source/host shutdown finishes.
            try:
                await task
            except (OSError, ValueError):
                pass
            raise


class FixtureConversation:
    async def respond(self, request):
        visual = request.environment.visual
        text = visual.description if visual else "No scene has been acquired."
        return GeneratedReply(
            f"[Offline fixture, no language inference] {text}", "fixture", "rover-v1"
        )


class FixtureVision:
    async def describe(self, image, *, previous=None):
        text = (
            "Synthetic steel-blue test image; this is fixture data."
            if image.source == "rover.synthetic"
            else "Image acquired; offline fixtures do not recognize its contents."
        )
        return SceneDescription(text, "fixture", "rover-vision-v1")


class FixtureInspection:
    async def evaluate(self, frame, questions):
        # Do not invent answers about a real image in offline mode.
        return JudgmentBatch(
            tuple(ImageJudgment(q, refused=True) for q in questions),
            "fixture",
            "no-image-judgment",
        )


class FixtureInterpreter:
    """Exact fixture cases only; never represented as a semantic model."""

    async def interpret(self, message):
        selected = {
            "what do you see?": Intent.INSPECT,
            "look around": Intent.INSPECT,
            "follow me": Intent.FOLLOW,
        }.get(message.text.strip().casefold(), Intent.CONVERSATION)
        return IntentJudgment(
            selected,
            tuple((i, float(i == selected)) for i in Intent),
            1.0,
            "fixture",
            "exact-cases-v1",
        )


class AssistantBridge:
    def __init__(
        self,
        *,
        camera=None,
        conversation=None,
        vision=None,
        interpreter=None,
        inspection=None,
        live=False,
    ):
        self.camera = camera or SyntheticCamera()
        self.session = ConversationSession(conversation or FixtureConversation())
        self.vision = vision or FixtureVision()
        self.router = RequestRouter(interpreter or FixtureInterpreter())
        self.inspector = ImageInspector(inspection or FixtureInspection())
        self.runner = AssistantRunner(self.session, router=self.router)
        self.lock = asyncio.Lock()
        self.live = live

    def update_telemetry(self, telemetry):
        at = (
            datetime.now(UTC)
            if telemetry["simulated"]
            else datetime.fromisoformat(telemetry["host_received_at"])
        )
        source = (
            "rover.simulated" if telemetry["simulated"] else "rover.controller-receipt"
        )
        for key, unit in (
            ("left_mps", "m/s"),
            ("right_mps", "m/s"),
            ("x_m", "m"),
            ("y_m", "m"),
            ("heading_rad", "rad"),
        ):
            self.session.state.observe(
                SensorObservation(source, at, key, telemetry[key], unit)
            )
        self.session.state.observe(
            SensorObservation(source, at, "bumper", int(telemetry["bumper"]), "boolean")
        )

    async def ask(self, text):
        async with self.lock:
            results = []

            async def observations():
                yield TextObservation("rover.operator", datetime.now(UTC), text)

            async def receive(result):
                results.append(result)

            await self.runner.run(observations(), on_result=receive)
            return results[0]

    async def inspect(self, question, *, describe=False):
        async with self.lock:
            frame = await self.camera.read()
            # Direct judgments do not wait for or require a generated caption.
            result = await self.inspector.inspect(
                frame,
                (InspectionQuestion("inspection", question),),
            )
            visual = None
            if describe:
                visual = await perceive_image(frame, self.vision)
                self.session.state.observe(visual)
            return {
                "scene": visual,
                "judgment": result.to_record(at=datetime.now(UTC)),
                "live_providers": self.live,
                "camera_source": frame.source,
            }
