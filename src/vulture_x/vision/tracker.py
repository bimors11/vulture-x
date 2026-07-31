"""OpenCV trackers with a stable Vulture-X result contract."""

from typing import Any, Protocol

import cv2
import numpy as np

from vulture_x.models import TrackingResult
from vulture_x.vision.target_state import tracking_result_from_bbox
from vulture_x.vision.video_source import BoundingBox, ImageFrame, VideoFrame


class BboxTracker(Protocol):
    def init(self, frame: ImageFrame, bbox: tuple[int, int, int, int]) -> bool:
        ...

    def update(self, frame: ImageFrame) -> tuple[bool, tuple[int, int, int, int]]:
        ...


class TemplateMatchingTracker:
    """Small ROI tracker adapted from the visual-nav fixed-wing prototype."""

    def __init__(
        self,
        *,
        search_margin_px: int = 120,
        minimum_match_score: float = 0.35,
        template_update_alpha: float = 0.10,
    ) -> None:
        self._search_margin_px = search_margin_px
        self._minimum_match_score = minimum_match_score
        self._template_update_alpha = template_update_alpha
        self._template: ImageFrame | None = None
        self._bbox: tuple[int, int, int, int] | None = None

    def init(self, frame: ImageFrame, bbox: tuple[int, int, int, int]) -> bool:
        x, y, width, height = bbox
        if width <= 0 or height <= 0:
            return False
        if x < 0 or y < 0 or x + width > frame.shape[1] or y + height > frame.shape[0]:
            return False

        self._bbox = (x, y, width, height)
        self._template = frame[y : y + height, x : x + width].copy()
        return self._template.size > 0

    def update(self, frame: ImageFrame) -> tuple[bool, tuple[int, int, int, int]]:
        if self._template is None or self._bbox is None:
            return False, (0, 0, 0, 0)

        x, y, width, height = self._bbox
        margin = self._search_margin_px
        x1 = max(0, x - margin)
        y1 = max(0, y - margin)
        x2 = min(frame.shape[1], x + width + margin)
        y2 = min(frame.shape[0], y + height + margin)
        search_image = frame[y1:y2, x1:x2]

        if search_image.shape[0] < height or search_image.shape[1] < width:
            return False, self._bbox

        uniform_template = bool(np.max(np.std(self._template, axis=(0, 1))) < 1.0)
        method = cv2.TM_SQDIFF if uniform_template else cv2.TM_CCOEFF_NORMED
        result = cv2.matchTemplate(search_image, self._template, method)
        min_value, max_value, min_location, max_location = cv2.minMaxLoc(result)
        if uniform_template:
            max_error = 255.0 * 255.0 * self._template.size
            match_score = 1.0 - (min_value / max_error)
            match_location = min_location
        else:
            match_score = max_value
            match_location = max_location
        if match_score < self._minimum_match_score:
            return False, self._bbox

        new_x = x1 + match_location[0]
        new_y = y1 + match_location[1]
        self._bbox = (new_x, new_y, width, height)

        new_template = frame[new_y : new_y + height, new_x : new_x + width].copy()
        if new_template.shape == self._template.shape:
            self._template = cv2.addWeighted(
                self._template,
                1.0 - self._template_update_alpha,
                new_template,
                self._template_update_alpha,
                0,
            ).astype(np.uint8, copy=False)

        return True, self._bbox


class OpenCvTracker:
    def __init__(self, tracker_name: str) -> None:
        if tracker_name not in {"CSRT", "KCF", "TEMPLATE"}:
            raise ValueError(f"unsupported tracker: {tracker_name}")
        self._tracker_name = tracker_name
        self._tracker: BboxTracker | Any | None = None
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
        if self._tracker_name == "TEMPLATE":
            return TemplateMatchingTracker()
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
