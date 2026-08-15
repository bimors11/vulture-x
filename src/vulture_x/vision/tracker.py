"""OpenCV trackers with a stable Vulture-X result contract."""

from dataclasses import dataclass
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


@dataclass(frozen=True, slots=True)
class TargetDetection:
    bbox: tuple[int, int, int, int]
    label: str = "target"
    confidence: float | None = None


@dataclass(frozen=True, slots=True)
class InitialTargetSelection:
    bbox: tuple[int, int, int, int]
    label: str
    confidence: float | None
    reason: str


def clamp_bbox(
    bbox: tuple[int, int, int, int],
    frame_width: int,
    frame_height: int,
) -> tuple[int, int, int, int]:
    x, y, width, height = bbox
    x = max(0, min(x, frame_width - 1))
    y = max(0, min(y, frame_height - 1))
    width = max(1, min(width, frame_width - x))
    height = max(1, min(height, frame_height - y))
    return x, y, width, height


def expand_bbox(
    bbox: tuple[int, int, int, int],
    padding_fraction: float,
    frame_width: int,
    frame_height: int,
) -> tuple[int, int, int, int]:
    x, y, width, height = bbox
    x -= int(width * padding_fraction)
    y -= int(height * padding_fraction)
    width += int(2 * width * padding_fraction)
    height += int(2 * height * padding_fraction)
    return clamp_bbox((x, y, width, height), frame_width, frame_height)


def choose_initial_bbox_from_point(
    frame: ImageFrame,
    point: tuple[int, int],
    detections: list[TargetDetection] | tuple[TargetDetection, ...] = (),
    *,
    bbox_padding_fraction: float = 0.18,
    search_radius_px: int = 80,
    fallback_size_px: int = 24,
) -> InitialTargetSelection:
    """Choose a tracker ROI from a clicked point using RDV's safe fallback order."""
    frame_height, frame_width = frame.shape[:2]
    containing = [
        detection for detection in detections if _bbox_contains_point(detection.bbox, point)
    ]
    if containing:
        chosen = min(containing, key=lambda detection: _bbox_area(detection.bbox))
        return InitialTargetSelection(
            bbox=expand_bbox(chosen.bbox, bbox_padding_fraction, frame_width, frame_height),
            label=chosen.label,
            confidence=chosen.confidence,
            reason="detector-hit",
        )

    nearest = _nearest_detection(point, detections, search_radius_px)
    if nearest is not None:
        return InitialTargetSelection(
            bbox=expand_bbox(nearest.bbox, bbox_padding_fraction, frame_width, frame_height),
            label=nearest.label,
            confidence=nearest.confidence,
            reason="detector-near",
        )

    fallback_bbox = _square_bbox_around_point(point, fallback_size_px, frame_width, frame_height)
    return InitialTargetSelection(
        bbox=fallback_bbox,
        label="manual",
        confidence=None,
        reason="fallback-square",
    )


def _bbox_contains_point(bbox: tuple[int, int, int, int], point: tuple[int, int]) -> bool:
    x, y, width, height = bbox
    point_x, point_y = point
    return x <= point_x <= x + width and y <= point_y <= y + height


def _bbox_area(bbox: tuple[int, int, int, int]) -> int:
    return bbox[2] * bbox[3]


def _nearest_detection(
    point: tuple[int, int],
    detections: list[TargetDetection] | tuple[TargetDetection, ...],
    search_radius_px: int,
) -> TargetDetection | None:
    point_x, point_y = point
    radius2 = search_radius_px * search_radius_px
    nearest: tuple[float, TargetDetection] | None = None
    for detection in detections:
        x, y, width, height = detection.bbox
        center_x = x + width / 2.0
        center_y = y + height / 2.0
        dx = center_x - point_x
        dy = center_y - point_y
        distance2 = dx * dx + dy * dy
        if distance2 <= radius2 and (nearest is None or distance2 < nearest[0]):
            nearest = (distance2, detection)
    return nearest[1] if nearest is not None else None


def _square_bbox_around_point(
    point: tuple[int, int],
    size_px: int,
    frame_width: int,
    frame_height: int,
) -> tuple[int, int, int, int]:
    point_x, point_y = point
    half = size_px // 2
    return clamp_bbox((point_x - half, point_y - half, size_px, size_px), frame_width, frame_height)


class TemplateMatchingTracker:
    """Small ROI tracker adapted from the visual-nav fixed-wing prototype."""

    def __init__(
        self,
        *,
        search_margin_px: int = 120,
        minimum_match_score: float = 0.35,
        template_update_alpha: float = 0.10,
        bbox_update_alpha: float = 0.35,
        max_center_jump_norm: float = 0.22,
        max_size_ratio: float = 8.0,
        scale_factors: tuple[float, ...] = (0.75, 0.85, 0.93, 1.0, 1.08, 1.18, 1.30),
    ) -> None:
        self._search_margin_px = search_margin_px
        self._minimum_match_score = minimum_match_score
        self._template_update_alpha = template_update_alpha
        self._bbox_update_alpha = bbox_update_alpha
        self._max_center_jump_norm = max_center_jump_norm
        self._max_size_ratio = max_size_ratio
        self._scale_factors = scale_factors
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

        match = self._best_match(search_image)
        if match is None:
            return False, self._bbox
        match_score, match_location, matched_width, matched_height = match
        if match_score < self._minimum_match_score:
            return False, self._bbox

        local_bbox = self._foreground_bbox(
            search_image,
            match_location,
            matched_width,
            matched_height,
        ) or (match_location[0], match_location[1], matched_width, matched_height)
        local_x, local_y, local_width, local_height = local_bbox
        desired_bbox = clamp_bbox(
            (x1 + local_x, y1 + local_y, local_width, local_height),
            frame.shape[1],
            frame.shape[0],
        )
        if not self._is_plausible_bbox_transition(self._bbox, desired_bbox, frame):
            return False, self._bbox
        self._bbox = self._smooth_bbox(self._bbox, desired_bbox)

        new_x, new_y, new_width, new_height = self._bbox
        new_template = frame[new_y : new_y + new_height, new_x : new_x + new_width].copy()
        if new_template.size > 0:
            old_template = cv2.resize(
                self._template,
                (new_template.shape[1], new_template.shape[0]),
                interpolation=cv2.INTER_LINEAR,
            )
            self._template = cv2.addWeighted(
                old_template,
                1.0 - self._template_update_alpha,
                new_template,
                self._template_update_alpha,
                0,
            ).astype(np.uint8, copy=False)

        return True, self._bbox

    def _is_plausible_bbox_transition(
        self,
        previous: tuple[int, int, int, int],
        desired: tuple[int, int, int, int],
        frame: ImageFrame,
    ) -> bool:
        px, py, pw, ph = previous
        dx, dy, dw, dh = desired
        previous_center_x = px + pw / 2.0
        previous_center_y = py + ph / 2.0
        desired_center_x = dx + dw / 2.0
        desired_center_y = dy + dh / 2.0
        frame_diag = float(np.hypot(frame.shape[1], frame.shape[0]))
        center_jump = float(
            np.hypot(desired_center_x - previous_center_x, desired_center_y - previous_center_y)
        )
        previous_area = max(1.0, float(pw * ph))
        desired_area = max(1.0, float(dw * dh))
        size_ratio = max(previous_area / desired_area, desired_area / previous_area)
        return (
            center_jump / max(1.0, frame_diag) <= self._max_center_jump_norm
            and size_ratio <= self._max_size_ratio
        )

    def _best_match(
        self,
        search_image: ImageFrame,
    ) -> tuple[float, tuple[int, int], int, int] | None:
        if self._template is None:
            return None
        uniform_template = bool(np.max(np.std(self._template, axis=(0, 1))) < 1.0)
        method = cv2.TM_SQDIFF if uniform_template else cv2.TM_CCOEFF_NORMED
        best: tuple[float, tuple[int, int], int, int] | None = None
        for scale in self._scale_factors:
            scaled_width = max(2, round(self._template.shape[1] * scale))
            scaled_height = max(2, round(self._template.shape[0] * scale))
            if search_image.shape[1] < scaled_width or search_image.shape[0] < scaled_height:
                continue
            scaled_template = cv2.resize(
                self._template,
                (scaled_width, scaled_height),
                interpolation=cv2.INTER_LINEAR,
            )
            result = cv2.matchTemplate(search_image, scaled_template, method)
            min_value, max_value, min_location, max_location = cv2.minMaxLoc(result)
            if uniform_template:
                max_error = 255.0 * 255.0 * scaled_template.size
                score = 1.0 - (min_value / max_error)
                location = (int(min_location[0]), int(min_location[1]))
            else:
                score = max_value
                location = (int(max_location[0]), int(max_location[1]))
            if best is None or score > best[0]:
                best = (score, location, scaled_width, scaled_height)
        return best

    def _foreground_bbox(
        self,
        search_image: ImageFrame,
        match_location: tuple[int, int],
        matched_width: int,
        matched_height: int,
    ) -> tuple[int, int, int, int] | None:
        if self._template is None:
            return None
        mean_color = np.mean(self._template.reshape(-1, self._template.shape[2]), axis=0)
        color_std = float(np.max(np.std(self._template, axis=(0, 1))))
        threshold = max(35.0, 20.0 + (4.0 * color_std))
        distance = np.linalg.norm(search_image.astype(np.float32) - mean_color, axis=2)
        mask = (distance <= threshold).astype(np.uint8) * 255
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        center_x = match_location[0] + matched_width / 2.0
        center_y = match_location[1] + matched_height / 2.0
        minimum_area = max(4.0, matched_width * matched_height * 0.35)
        previous_width = self._bbox[2] if self._bbox is not None else matched_width
        previous_height = self._bbox[3] if self._bbox is not None else matched_height
        max_width = max(matched_width * 2.2, previous_width * 3.0)
        max_height = max(matched_height * 2.2, previous_height * 3.0)
        candidates: list[tuple[float, tuple[int, int, int, int]]] = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < minimum_area:
                continue
            x, y, width, height = cv2.boundingRect(contour)
            if width > max_width or height > max_height:
                continue
            contains_center = x <= center_x <= x + width and y <= center_y <= y + height
            contour_center_x = x + width / 2.0
            contour_center_y = y + height / 2.0
            distance2 = (contour_center_x - center_x) ** 2 + (contour_center_y - center_y) ** 2
            score = distance2 if contains_center else distance2 + 1_000_000.0
            candidates.append((score, (x, y, width, height)))
        if not candidates:
            return None
        return min(candidates, key=lambda item: item[0])[1]

    def _smooth_bbox(
        self,
        previous: tuple[int, int, int, int],
        desired: tuple[int, int, int, int],
    ) -> tuple[int, int, int, int]:
        px, py, pw, ph = previous
        dx, dy, dw, dh = desired
        alpha = self._bbox_update_alpha
        width = max(2, round(pw + (dw - pw) * alpha))
        height = max(2, round(ph + (dh - ph) * alpha))
        previous_center_x = px + pw / 2.0
        previous_center_y = py + ph / 2.0
        desired_center_x = dx + dw / 2.0
        desired_center_y = dy + dh / 2.0
        center_x = previous_center_x + (desired_center_x - previous_center_x) * alpha
        center_y = previous_center_y + (desired_center_y - previous_center_y) * alpha
        return (
            round(center_x - width / 2.0),
            round(center_y - height / 2.0),
            width,
            height,
        )


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

    def initialize_from_point(
        self,
        frame: VideoFrame,
        point: tuple[int, int],
        detections: list[TargetDetection] | tuple[TargetDetection, ...] = (),
    ) -> InitialTargetSelection:
        selection = choose_initial_bbox_from_point(frame.image, point, detections)
        self.initialize(frame, _bbox_to_float(selection.bbox))
        return selection

    def relock_from_detections(
        self,
        frame: VideoFrame,
        detections: list[TargetDetection] | tuple[TargetDetection, ...],
    ) -> InitialTargetSelection | None:
        if not detections:
            return None
        frame_height, frame_width = frame.image.shape[:2]
        chosen = max(detections, key=lambda detection: detection.confidence or 0.0)
        selection = InitialTargetSelection(
            bbox=expand_bbox(chosen.bbox, 0.18, frame_width, frame_height),
            label=chosen.label,
            confidence=chosen.confidence,
            reason="detector-relock",
        )
        self.initialize(frame, _bbox_to_float(selection.bbox))
        return selection

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


def _bbox_to_float(bbox: tuple[int, int, int, int]) -> BoundingBox:
    x, y, width, height = bbox
    return float(x), float(y), float(width), float(height)
