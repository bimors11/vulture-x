"""Tracking confidence and loss timeout policy."""

from vulture_x.config import SafetyConfig
from vulture_x.enums import SafetyReason, SafetyStatus
from vulture_x.models import SafetyResult, TrackingResult


class TrackingSafetySupervisor:
    def __init__(self, config: SafetyConfig) -> None:
        self._config = config
        self._invalid_since_monotonic_s: float | None = None

    def evaluate(
        self,
        tracking: TrackingResult,
        *,
        now_monotonic_s: float,
    ) -> SafetyResult:
        valid = (
            tracking.detected
            and tracking.confidence >= self._config.minimum_tracking_confidence
        )
        if valid:
            self._invalid_since_monotonic_s = None
            return SafetyResult(SafetyStatus.NORMAL, SafetyReason.NONE, {})

        if self._invalid_since_monotonic_s is None:
            self._invalid_since_monotonic_s = now_monotonic_s
        invalid_age = now_monotonic_s - self._invalid_since_monotonic_s
        reason = (
            SafetyReason.TARGET_NOT_DETECTED
            if not tracking.detected
            else SafetyReason.TRACKING_CONFIDENCE_LOW
        )
        details: dict[str, object] = {
            "invalid_age_s": invalid_age,
            "confidence": tracking.confidence,
        }
        if invalid_age >= self._config.tracking_abort_timeout_s:
            return SafetyResult(SafetyStatus.ABORT_REQUIRED, SafetyReason.TRACKING_TIMEOUT, details)
        if invalid_age >= self._config.tracking_warning_timeout_s:
            return SafetyResult(SafetyStatus.HOLD_REQUIRED, reason, details)
        return SafetyResult(SafetyStatus.WARNING, reason, details)

    def reset(self) -> None:
        self._invalid_since_monotonic_s = None
