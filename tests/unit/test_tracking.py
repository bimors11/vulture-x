import numpy as np
import pytest

from vulture_x.vision.target_state import tracking_result_from_bbox
from vulture_x.vision.tracker import OpenCvTracker, TemplateMatchingTracker
from vulture_x.vision.video_source import VideoFrame


def test_bbox_is_normalized_to_image_errors() -> None:
    result = tracking_result_from_bbox(
        timestamp_monotonic_s=1.0,
        frame_width=640,
        frame_height=480,
        bbox=(480.0, 120.0, 64.0, 48.0),
        confidence=0.8,
    )
    assert result.detected
    assert result.center_x_normalized == pytest.approx(0.8)
    assert result.center_y_normalized == pytest.approx(0.3)
    assert result.horizontal_error == pytest.approx(0.6)
    assert result.vertical_error == pytest.approx(-0.4)
    assert -1 <= result.horizontal_error <= 1
    assert -1 <= result.vertical_error <= 1


@pytest.mark.parametrize(
    "bbox",
    [None, (-1.0, 0.0, 20.0, 20.0), (0.0, 0.0, 0.0, 20.0), (630.0, 0.0, 20.0, 20.0)],
)
def test_missing_or_invalid_bbox_reports_target_lost(
    bbox: tuple[float, float, float, float] | None,
) -> None:
    result = tracking_result_from_bbox(
        timestamp_monotonic_s=1.0,
        frame_width=640,
        frame_height=480,
        bbox=bbox,
        confidence=1.0,
    )
    assert not result.detected
    assert result.confidence == 0.0
    assert result.horizontal_error == 0.0


def test_template_tracker_follows_selected_patch_motion() -> None:
    first = np.zeros((120, 160, 3), dtype=np.uint8)
    second = np.zeros_like(first)
    first[30:50, 40:70] = (20, 200, 255)
    second[44:64, 58:88] = (20, 200, 255)

    tracker = OpenCvTracker("TEMPLATE")
    tracker.initialize(VideoFrame(1.0, 1, first), (40.0, 30.0, 30.0, 20.0))

    result = tracker.update(VideoFrame(2.0, 2, second))

    assert result.detected
    assert result.center_x_normalized == pytest.approx((58 + 15) / 160)
    assert result.center_y_normalized == pytest.approx((44 + 10) / 120)
    assert result.confidence == 1.0


def test_template_tracker_rejects_low_match_score() -> None:
    first = np.zeros((120, 160, 3), dtype=np.uint8)
    first[30:50, 40:70] = (20, 200, 255)
    second = np.zeros_like(first)

    tracker = TemplateMatchingTracker(minimum_match_score=0.99)
    assert tracker.init(first, (40, 30, 30, 20))

    detected, bbox = tracker.update(second)

    assert not detected
    assert bbox == (40, 30, 30, 20)
