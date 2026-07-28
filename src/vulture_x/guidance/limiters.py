"""Speed, acceleration, vertical-speed, and yaw-rate command limiters."""

from dataclasses import replace
from math import sqrt

from vulture_x.config import GuidanceConfig, SafetyConfig
from vulture_x.enums import LimiterDecision
from vulture_x.models import GuidanceCommand, LimitedCommand


class CommandLimiter:
    def __init__(self, guidance: GuidanceConfig, safety: SafetyConfig) -> None:
        self._guidance = guidance
        self._safety = safety
        self._previous: GuidanceCommand | None = None

    def apply(
        self,
        command: GuidanceCommand,
        *,
        now_monotonic_s: float,
    ) -> LimitedCommand:
        if not command.is_valid_at(now_monotonic_s):
            return LimitedCommand(command, LimiterDecision.REJECTED, ("expired",))

        modified: set[str] = set()
        forward = self._clamp(
            command.velocity_forward_mps,
            -self._safety.maximum_test_speed_mps,
            self._safety.maximum_test_speed_mps,
            "velocity_forward_mps",
            modified,
        )
        right = self._clamp(
            command.velocity_right_mps,
            -self._guidance.max_lateral_speed_mps,
            self._guidance.max_lateral_speed_mps,
            "velocity_right_mps",
            modified,
        )
        down = self._clamp(
            command.velocity_down_mps,
            -self._guidance.max_vertical_speed_mps,
            self._guidance.max_vertical_speed_mps,
            "velocity_down_mps",
            modified,
        )
        yaw_rate = self._clamp(
            command.yaw_rate_deg_s,
            -self._guidance.max_yaw_rate_deg_s,
            self._guidance.max_yaw_rate_deg_s,
            "yaw_rate_deg_s",
            modified,
        )

        if self._previous is not None:
            delta_t = command.timestamp_monotonic_s - self._previous.timestamp_monotonic_s
            if delta_t > 0:
                max_delta = self._guidance.max_acceleration_mps2 * delta_t
                deltas = (
                    forward - self._previous.velocity_forward_mps,
                    right - self._previous.velocity_right_mps,
                    down - self._previous.velocity_down_mps,
                )
                delta_magnitude = sqrt(sum(delta * delta for delta in deltas))
                if delta_magnitude > max_delta:
                    scale = max_delta / delta_magnitude
                    forward = self._previous.velocity_forward_mps + deltas[0] * scale
                    right = self._previous.velocity_right_mps + deltas[1] * scale
                    down = self._previous.velocity_down_mps + deltas[2] * scale
                    modified.add("acceleration")

        limited = replace(
            command,
            velocity_forward_mps=forward,
            velocity_right_mps=right,
            velocity_down_mps=down,
            yaw_rate_deg_s=yaw_rate,
        )
        self._previous = limited
        decision = LimiterDecision.MODIFIED if modified else LimiterDecision.ACCEPTED
        return LimitedCommand(limited, decision, tuple(sorted(modified)))

    @staticmethod
    def _clamp(
        value: float,
        minimum: float,
        maximum: float,
        field: str,
        modified: set[str],
    ) -> float:
        limited = min(max(value, minimum), maximum)
        if limited != value:
            modified.add(field)
        return limited

    def reset(self) -> None:
        self._previous = None

