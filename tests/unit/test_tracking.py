import pytest

from vulture_x.vision.target_state import tracking_result_from_bbox


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

