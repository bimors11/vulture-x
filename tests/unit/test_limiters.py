import pytest

from vulture_x.config import GuidanceConfig, SafetyConfig
from vulture_x.enums import LimiterDecision
from vulture_x.guidance.limiters import CommandLimiter
from vulture_x.models import GuidanceCommand


def make_command(
    timestamp: float,
    forward: float,
    right: float = 0.0,
    down: float = 0.0,
    yaw: float = 0.0,
) -> GuidanceCommand:
    return GuidanceCommand(timestamp, forward, right, down, yaw, timestamp + 0.25, "test")


def test_velocity_and_yaw_limits_are_enforced() -> None:
    limiter = CommandLimiter(GuidanceConfig(), SafetyConfig())
    result = limiter.apply(
        make_command(1.0, 9.0, 8.0, -7.0, 50.0),
        now_monotonic_s=1.0,
    )
    assert result.decision is LimiterDecision.MODIFIED
    assert result.command.velocity_forward_mps == 5.0
    assert result.command.velocity_right_mps == 3.0
    assert result.command.velocity_down_mps == -2.0
    assert result.command.yaw_rate_deg_s == 30.0


def test_acceleration_limit_is_vector_bounded() -> None:
    limiter = CommandLimiter(GuidanceConfig(max_acceleration_mps2=1.5), SafetyConfig())
    limiter.apply(make_command(1.0, 0.0), now_monotonic_s=1.0)
    result = limiter.apply(make_command(2.0, 5.0), now_monotonic_s=2.0)
    assert result.command.velocity_forward_mps == pytest.approx(1.5)
    assert "acceleration" in result.modified_fields


def test_expired_command_is_rejected() -> None:
    limiter = CommandLimiter(GuidanceConfig(), SafetyConfig())
    result = limiter.apply(make_command(1.0, 1.0), now_monotonic_s=1.3)
    assert result.decision is LimiterDecision.REJECTED
    assert result.modified_fields == ("expired",)

