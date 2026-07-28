import pytest

from vulture_x.config import GuidanceConfig
from vulture_x.guidance.image_guidance import ImageGuidanceController
from vulture_x.models import TrackingResult


def tracking(horizontal: float, vertical: float, size: float = 0.05) -> TrackingResult:
    return TrackingResult(1.0, True, 1.0, 0.5, 0.5, size, size, horizontal, vertical)


def test_guidance_deadband_zeroes_image_correction() -> None:
    controller = ImageGuidanceController(GuidanceConfig(), 0.25)
    command = controller.calculate(tracking(0.04, -0.04), now_monotonic_s=10.0)
    assert command.velocity_right_mps == 0
    assert command.velocity_down_mps == 0
    assert command.yaw_rate_deg_s == 0
    assert command.valid_until_monotonic_s == 10.25


def test_guidance_maps_image_error_and_target_size() -> None:
    controller = ImageGuidanceController(GuidanceConfig(), 0.25)
    command = controller.calculate(tracking(0.5, -0.5), now_monotonic_s=10.0)
    assert command.velocity_right_mps == pytest.approx(1.5)
    assert command.velocity_down_mps == pytest.approx(-1.0)
    assert command.yaw_rate_deg_s == pytest.approx(15.0)
    assert command.velocity_forward_mps > 0


def test_lost_target_generates_zero_pursuit() -> None:
    controller = ImageGuidanceController(GuidanceConfig(), 0.25)
    lost = TrackingResult(1.0, False, 0.0, 0.5, 0.5, 0, 0, 0, 0)
    command = controller.calculate(lost, now_monotonic_s=10.0)
    assert command.velocity_forward_mps == 0
    assert command.velocity_right_mps == 0
    assert command.reason == "tracking_invalid"

