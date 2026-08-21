"""Frame source implementations for legacy and in-memory capture paths."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any, Protocol, cast

import cv2
import numpy as np
from numpy.typing import NDArray

from vulture_x.runtime.state import VideoFrame

STABLE_FRAME_MIN_AGE_S = 0.02


class FrameSource(Protocol):
    def read(self) -> VideoFrame | None:
        ...

    def close(self) -> None:
        ...


class FileFrameSource:
    """Legacy latest-JPEG/PNG directory reader."""

    def __init__(self, camera_dir: Path, *, source_name: str = "file") -> None:
        self._camera_dir = camera_dir
        self._source_name = source_name
        self._sequence = 0
        self._last_key: tuple[str, int, int] | None = None

    def read(self) -> VideoFrame | None:
        image_path = newest_stable_image(self._camera_dir)
        if image_path is None:
            return None
        try:
            stat = image_path.stat()
        except FileNotFoundError:
            return None
        key = (str(image_path), stat.st_mtime_ns, stat.st_size)
        if key == self._last_key:
            return None
        frame = read_stable_image(image_path)
        if frame is None:
            return None
        self._last_key = key
        self._sequence += 1
        return VideoFrame(
            sequence=self._sequence,
            image=frame,
            received_timestamp_ns=time.monotonic_ns(),
            source=f"{self._source_name}:{image_path}",
        )

    def close(self) -> None:
        return


class OpenCvCaptureSource:
    def __init__(self, capture: cv2.VideoCapture, *, source_name: str = "opencv") -> None:
        self._capture = capture
        self._source_name = source_name
        self._sequence = 0

    def read(self) -> VideoFrame | None:
        ok, frame = self._capture.read()
        if not ok or frame is None:
            return None
        image = cast(NDArray[np.uint8], frame)
        self._sequence += 1
        return VideoFrame(
            sequence=self._sequence,
            image=image,
            received_timestamp_ns=time.monotonic_ns(),
            source=self._source_name,
        )

    def close(self) -> None:
        self._capture.release()


class GstAppSinkFrameSource:
    """GStreamer appsink source using PyGObject bindings.

    This path is used when system GStreamer/appsink is available even if the
    installed OpenCV wheel lacks CAP_GSTREAMER support.
    """

    def __init__(self, pipeline: str, *, source_name: str = "appsink") -> None:
        self._source_name = source_name
        self._sequence = 0
        self._gst = import_gst()
        self._gst.init(None)
        self._pipeline = self._gst.parse_launch(pipeline)
        sink = self._pipeline.get_by_name("sink")
        if sink is None:
            raise RuntimeError("appsink_pipeline_missing_named_sink")
        self._sink = sink
        self._pipeline.set_state(self._gst.State.PLAYING)

    def read(self) -> VideoFrame | None:
        sample = self._sink.emit("try-pull-sample", 20_000_000)
        if sample is None:
            return None
        buffer = sample.get_buffer()
        caps = sample.get_caps()
        if buffer is None or caps is None:
            return None
        structure = caps.get_structure(0)
        width = int(structure.get_value("width"))
        height = int(structure.get_value("height"))
        success, info = buffer.map(self._gst.MapFlags.READ)
        if not success:
            return None
        try:
            raw = np.frombuffer(info.data, dtype=np.uint8)
            image = raw.reshape((height, width, 3)).copy()
        finally:
            buffer.unmap(info)
        self._sequence += 1
        return VideoFrame(
            sequence=self._sequence,
            image=image,
            received_timestamp_ns=time.monotonic_ns(),
            source=self._source_name,
        )

    def close(self) -> None:
        self._pipeline.set_state(self._gst.State.NULL)


def appsink_pipeline_from_udp_h264(port: int = 5600, host: str = "127.0.0.1") -> str:
    return (
        f"udpsrc address={host} port={port} reuse=false "
        "caps=application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 "
        "! rtph264depay ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR "
        "! appsink name=sink max-buffers=1 drop=true sync=false emit-signals=false"
    )


def appsink_pipeline_from_rtsp(
    url: str,
    *,
    latency_ms: int = 100,
    protocols: str = "tcp",
    max_rate: int = 30,
) -> str:
    if not url.startswith(("rtsp://", "rtsps://")):
        raise ValueError("rtsp_url_required")
    protocol_value = protocols.lower()
    if protocol_value not in {"tcp", "udp"}:
        raise ValueError("invalid_rtsp_protocols")
    return (
        f"uridecodebin uri={url} source::latency={max(0, min(500, latency_ms))} "
        "source::drop-on-latency=true source::do-retransmission=false "
        f"source::protocols={protocol_value} "
        f"! videorate drop-only=true skip-to-first=true max-rate={max(1, min(60, max_rate))} "
        "! videoconvert ! video/x-raw,format=BGR "
        "! appsink name=sink max-buffers=1 drop=true sync=false emit-signals=false"
    )


def newest_stable_image(camera_dir: Path, *, now: float | None = None) -> Path | None:
    now = time.time() if now is None else now
    candidates: list[tuple[float, Path]] = []
    for pattern in ("*.png", "*.jpg", "*.jpeg"):
        for path in camera_dir.glob(pattern):
            try:
                stat = path.stat()
            except FileNotFoundError:
                continue
            if not path.is_file() or stat.st_size <= 0:
                continue
            if now - stat.st_mtime < STABLE_FRAME_MIN_AGE_S:
                continue
            candidates.append((stat.st_mtime, path))
    if not candidates:
        return None
    return max(candidates, key=lambda item: item[0])[1]


def read_stable_image(image_path: Path) -> Any | None:
    last_size = -1
    for _ in range(3):
        try:
            size = image_path.stat().st_size
        except FileNotFoundError:
            return None
        if size <= 0:
            time.sleep(0.01)
            continue
        frame = cv2.imread(str(image_path))
        if frame is not None and size == last_size:
            return frame
        last_size = size
        time.sleep(0.01)
    return cv2.imread(str(image_path))


def import_gst() -> Any:
    try:
        import gi  # type: ignore[import-untyped]

        gi.require_version("Gst", "1.0")
        from gi.repository import Gst  # type: ignore[import-untyped]
    except Exception as exc:
        raise RuntimeError("python_gstreamer_bindings_unavailable") from exc
    return Gst
