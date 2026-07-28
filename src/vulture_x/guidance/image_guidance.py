"""Conservative image-error to body-velocity guidance mapping."""

from math import sqrt

from vulture_x.config import GuidanceConfig
from vulture_x.models import GuidanceCommand, TrackingResult


class ImageGuidanceController:
    def __init__(self, config: GuidanceConfig, command_expiration_s: float) -> None:
        self._config = config
        self._command_expiration_s = command_expiration_s

    def calculate(
        self,
        tracking: TrackingResult,
        *,
        now_monotonic_s: float,
    ) -> GuidanceCommand:
        if not tracking.detected:
            return self._zero_command(now_monotonic_s, "tracking_invalid")

        horizontal = self._deadband(
            tracking.horizontal_error,
            self._config.horizontal_deadband,
        )
        vertical = self._deadband(
            tracking.vertical_error,
            self._config.vertical_deadband,
        )
        apparent_size = sqrt(tracking.width_normalized * tracking.height_normalized)
        size_error = (self._config.target_size_setpoint - apparent_size) / (
            self._config.target_size_setpoint
        )
        return GuidanceCommand(
            timestamp_monotonic_s=now_monotonic_s,
            velocity_forward_mps=size_error * self._config.max_forward_speed_mps,
            velocity_right_mps=horizontal * self._config.max_lateral_speed_mps,
            velocity_down_mps=vertical * self._config.max_vertical_speed_mps,
            yaw_rate_deg_s=horizontal * self._config.max_yaw_rate_deg_s,
            valid_until_monotonic_s=now_monotonic_s + self._command_expiration_s,
            reason="image_guidance",
        )

    def _zero_command(self, now_monotonic_s: float, reason: str) -> GuidanceCommand:
        return GuidanceCommand(
            timestamp_monotonic_s=now_monotonic_s,
            velocity_forward_mps=0.0,
            velocity_right_mps=0.0,
            velocity_down_mps=0.0,
            yaw_rate_deg_s=0.0,
            valid_until_monotonic_s=now_monotonic_s + self._command_expiration_s,
            reason=reason,
        )

    @staticmethod
    def _deadband(value: float, deadband: float) -> float:
        return 0.0 if abs(value) <= deadband else value

