import numpy as np
import pytest

from vulture_x.vision.target_state import tracking_result_from_bbox
from vulture_x.vision.tracker import (
    OpenCvTracker,
    TargetDetection,
    TemplateMatchingTracker,
    choose_initial_bbox_from_point,
    expand_bbox,
)
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
    assert result.center_x_normalized == pytest.approx((46 + 15) / 160)
    assert result.center_y_normalized == pytest.approx((35 + 10) / 120)
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


def test_template_tracker_rejects_implausible_center_jump() -> None:
    first = np.zeros((120, 160, 3), dtype=np.uint8)
    second = np.zeros_like(first)
    first[30:50, 40:70] = (20, 200, 255)
    second[80:100, 120:150] = (20, 200, 255)

    tracker = TemplateMatchingTracker(
        minimum_match_score=0.20,
        max_center_jump_norm=0.10,
    )
    assert tracker.init(first, (40, 30, 30, 20))

    detected, bbox = tracker.update(second)

    assert not detected
    assert bbox == (40, 30, 30, 20)


def test_template_tracker_adapts_bbox_size_as_target_grows() -> None:
    first = np.zeros((140, 200, 3), dtype=np.uint8)
    second = np.zeros_like(first)
    first[45:61, 60:80] = (0, 80, 255)
    second[40:82, 54:108] = (0, 80, 255)

    tracker = TemplateMatchingTracker(
        minimum_match_score=0.20,
        bbox_update_alpha=0.50,
    )
    assert tracker.init(first, (60, 45, 20, 16))

    detected, bbox = tracker.update(second)

    assert detected
    assert bbox[2] > 20
    assert bbox[3] > 16
    assert 70 <= bbox[0] + bbox[2] / 2 <= 92
    assert 52 <= bbox[1] + bbox[3] / 2 <= 72


def test_template_tracker_rejects_ambiguous_uniform_background_lock() -> None:
    first = np.full((120, 160, 3), (10, 10, 10), dtype=np.uint8)
    second = np.full_like(first, (10, 10, 10))
    second[55:65, 70:82] = (200, 200, 200)

    tracker = TemplateMatchingTracker(
        minimum_match_score=0.20,
        bbox_update_alpha=0.50,
    )
    assert tracker.init(first, (60, 45, 20, 16))

    detected, bbox = tracker.update(second)

    assert not detected
    assert bbox == (60, 45, 20, 16)


def test_rdv_bbox_padding_is_clamped_to_frame() -> None:
    assert expand_bbox((2, 3, 20, 10), 0.25, 40, 30) == (0, 1, 30, 15)
    assert expand_bbox((30, 22, 10, 8), 0.50, 40, 30) == (25, 18, 15, 12)


def test_rdv_point_initialization_prefers_smallest_detection_containing_click() -> None:
    frame = np.zeros((120, 160, 3), dtype=np.uint8)

    selection = choose_initial_bbox_from_point(
        frame,
        (62, 42),
        (
            TargetDetection((40, 20, 80, 60), "large", 0.9),
            TargetDetection((55, 35, 20, 16), "small", 0.7),
        ),
    )

    assert selection.reason == "detector-hit"
    assert selection.label == "small"
    assert selection.confidence == 0.7
    assert selection.bbox == (52, 33, 27, 21)


def test_rdv_point_initialization_uses_nearest_detection_inside_search_radius() -> None:
    frame = np.zeros((120, 160, 3), dtype=np.uint8)

    selection = choose_initial_bbox_from_point(
        frame,
        (80, 60),
        (
            TargetDetection((120, 80, 20, 20), "far", 0.95),
            TargetDetection((88, 62, 12, 12), "near", 0.3),
        ),
        search_radius_px=30,
    )

    assert selection.reason == "detector-near"
    assert selection.label == "near"
    assert selection.bbox == (86, 60, 16, 16)


def test_rdv_point_initialization_falls_back_to_manual_square() -> None:
    frame = np.zeros((120, 160, 3), dtype=np.uint8)

    selection = choose_initial_bbox_from_point(frame, (4, 5), fallback_size_px=24)

    assert selection.reason == "fallback-square"
    assert selection.label == "manual"
    assert selection.confidence is None
    assert selection.bbox == (0, 0, 24, 24)


def test_opencv_tracker_can_initialize_from_rdv_point_selection() -> None:
    first = np.zeros((120, 160, 3), dtype=np.uint8)
    second = np.zeros_like(first)
    first[30:50, 40:70] = (20, 200, 255)
    second[44:64, 58:88] = (20, 200, 255)

    tracker = OpenCvTracker("TEMPLATE")
    selection = tracker.initialize_from_point(
        VideoFrame(1.0, 1, first),
        (52, 40),
        (TargetDetection((40, 30, 30, 20), "synthetic", 0.8),),
    )

    result = tracker.update(VideoFrame(2.0, 2, second))

    assert selection.reason == "detector-hit"
    assert result.detected
