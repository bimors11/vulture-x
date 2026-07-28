"""OpenCV CSRT/KCF tracker with a stable Vulture-X result contract."""

from typing import Any

import cv2

from vulture_x.models import TrackingResult
from vulture_x.vision.target_state import tracking_result_from_bbox
from vulture_x.vision.video_source import BoundingBox, VideoFrame


class OpenCvTracker:
    def __init__(self, tracker_name: str) -> None:
        if tracker_name not in {"CSRT", "KCF"}:
            raise ValueError(f"unsupported tracker: {tracker_name}")
        self._tracker_name = tracker_name
        self._tracker: Any | None = None
        self._initialized = False

    def initialize(self, frame: VideoFrame, bbox: BoundingBox) -> None:
        tracker = self._create_tracker()
        integer_bbox = tuple(round(value) for value in bbox)
        result = tracker.init(frame.image, integer_bbox)
        if result is False:
            raise ValueError("OpenCV rejected the initial target bounding box")
        self._tracker = tracker
        self._initialized = True

    def update(self, frame: VideoFrame) -> TrackingResult:
        if not self._initialized or self._tracker is None:
            raise RuntimeError("tracker must be initialized before update")
        detected, raw_bbox = self._tracker.update(frame.image)
        bbox: BoundingBox | None = None
        if detected:
            x, y, width, height = raw_bbox
            bbox = (float(x), float(y), float(width), float(height))
        height, width = frame.image.shape[:2]
        return tracking_result_from_bbox(
            timestamp_monotonic_s=frame.timestamp_monotonic_s,
            frame_width=width,
            frame_height=height,
            bbox=bbox,
            confidence=1.0 if detected else 0.0,
        )

    def reset(self) -> None:
        self._tracker = None
        self._initialized = False

    def _create_tracker(self) -> Any:
        direct_name = f"Tracker{self._tracker_name}_create"
        direct_factory = getattr(cv2, direct_name, None)
        if direct_factory is not None:
            return direct_factory()
        legacy = getattr(cv2, "legacy", None)
        legacy_factory = getattr(legacy, direct_name, None) if legacy is not None else None
        if legacy_factory is not None:
            return legacy_factory()
        raise RuntimeError(
            f"OpenCV {self._tracker_name} tracker is unavailable; "
            "install opencv-contrib-python"
        )
