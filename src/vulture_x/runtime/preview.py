"""Preview rendering helpers for in-memory runtime snapshots."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from typing import Any

import cv2

from vulture_x.runtime.state import RuntimeSnapshot, VideoFrame

HudRenderer = Callable[[Any, RuntimeSnapshot], None]


def encode_runtime_preview_jpeg(
    frame: VideoFrame,
    snapshot: RuntimeSnapshot,
    *,
    hud_renderer: HudRenderer | None = None,
    jpeg_quality: int = 85,
) -> bytes | None:
    """Render HUD on a native-size in-memory frame and encode JPEG once."""

    image = frame.image.copy()
    if hud_renderer is not None:
        hud_renderer(image, snapshot)
    quality = max(1, min(100, int(jpeg_quality)))
    ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    return encoded.tobytes() if ok else None


def runtime_preview_metadata(snapshot: RuntimeSnapshot) -> Mapping[str, object]:
    frame = snapshot.frame
    tracking = snapshot.tracking
    vehicle = snapshot.vehicle
    shape = None if frame is None else tuple(int(value) for value in frame.image.shape[:2])
    return {
        "frame_sequence": None if frame is None else frame.sequence,
        "frame_shape_hw": shape,
        "tracking_sequence": None if tracking is None else tracking.sequence,
        "tracking_frame_sequence": None if tracking is None else tracking.frame_sequence,
        "vehicle_mode": None if vehicle is None else vehicle.mode,
        "vehicle_armed": None if vehicle is None else vehicle.armed,
    }
