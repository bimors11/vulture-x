from vulture_x.config import GuidanceConfig, SafetyConfig
from vulture_x.enums import LimiterDecision
from vulture_x.guidance.image_guidance import ImageGuidanceController
from vulture_x.guidance.limiters import CommandLimiter
from vulture_x.vision.target_state import tracking_result_from_bbox


def test_tracker_contract_to_guidance_to_limiter() -> None:
    tracking = tracking_result_from_bbox(
        timestamp_monotonic_s=1.0,
        frame_width=640,
        frame_height=480,
        bbox=(500.0, 200.0, 30.0, 30.0),
        confidence=1.0,
    )
    guidance = ImageGuidanceController(GuidanceConfig(), 0.25)
    raw = guidance.calculate(tracking, now_monotonic_s=1.0)
    result = CommandLimiter(GuidanceConfig(), SafetyConfig()).apply(
        raw,
        now_monotonic_s=1.0,
    )
    assert result.decision in {LimiterDecision.ACCEPTED, LimiterDecision.MODIFIED}
    assert abs(result.command.velocity_forward_mps) <= 5.0
    assert abs(result.command.velocity_right_mps) <= 3.0
    assert abs(result.command.velocity_down_mps) <= 2.0

