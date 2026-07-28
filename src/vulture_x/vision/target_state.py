"""Bounding-box validation and normalized tracking-result construction."""

from vulture_x.models import TrackingResult
from vulture_x.vision.video_source import BoundingBox


def tracking_result_from_bbox(
    *,
    timestamp_monotonic_s: float,
    frame_width: int,
    frame_height: int,
    bbox: BoundingBox | None,
    confidence: float,
) -> TrackingResult:
    if bbox is None or not _bbox_valid(bbox, frame_width, frame_height):
        return TrackingResult(
            timestamp_monotonic_s=timestamp_monotonic_s,
            detected=False,
            confidence=0.0,
            center_x_normalized=0.5,
            center_y_normalized=0.5,
            width_normalized=0.0,
            height_normalized=0.0,
            horizontal_error=0.0,
            vertical_error=0.0,
        )

    x, y, width, height = bbox
    center_x = (x + width / 2) / frame_width
    center_y = (y + height / 2) / frame_height
    return TrackingResult(
        timestamp_monotonic_s=timestamp_monotonic_s,
        detected=True,
        confidence=min(max(confidence, 0.0), 1.0),
        center_x_normalized=center_x,
        center_y_normalized=center_y,
        width_normalized=width / frame_width,
        height_normalized=height / frame_height,
        horizontal_error=(2 * center_x) - 1,
        vertical_error=(2 * center_y) - 1,
    )


def _bbox_valid(bbox: BoundingBox, frame_width: int, frame_height: int) -> bool:
    x, y, width, height = bbox
    return (
        width > 1
        and height > 1
        and x >= 0
        and y >= 0
        and x + width <= frame_width
        and y + height <= frame_height
    )

