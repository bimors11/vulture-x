"""Telemetry freshness monitors used before the real MAVLink client exists."""

from vulture_x.enums import SafetyReason, SafetyStatus
from vulture_x.models import SafetyResult


class HeartbeatMonitor:
    def __init__(self, timeout_s: float) -> None:
        self._timeout_s = timeout_s
        self._last_heartbeat_monotonic_s: float | None = None

    def record(self, timestamp_monotonic_s: float) -> None:
        self._last_heartbeat_monotonic_s = timestamp_monotonic_s

    def evaluate(self, now_monotonic_s: float) -> SafetyResult:
        if self._last_heartbeat_monotonic_s is None:
            return SafetyResult(
                SafetyStatus.ABORT_REQUIRED,
                SafetyReason.MAVLINK_HEARTBEAT_TIMEOUT,
                {"age_s": None, "timeout_s": self._timeout_s},
            )
        age = now_monotonic_s - self._last_heartbeat_monotonic_s
        if age > self._timeout_s:
            return SafetyResult(
                SafetyStatus.ABORT_REQUIRED,
                SafetyReason.MAVLINK_HEARTBEAT_TIMEOUT,
                {"age_s": age, "timeout_s": self._timeout_s},
            )
        return SafetyResult(SafetyStatus.NORMAL, SafetyReason.NONE, {"age_s": age})

