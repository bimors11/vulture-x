from vulture_x.config import SafetyConfig
from vulture_x.enums import SafetyReason, SafetyStatus
from vulture_x.models import TrackingResult
from vulture_x.safety.tracking_safety import TrackingSafetySupervisor
from vulture_x.vehicle.telemetry import HeartbeatMonitor


def lost_tracking() -> TrackingResult:
    return TrackingResult(0.0, False, 0.0, 0.5, 0.5, 0, 0, 0, 0)


def test_tracking_loss_escalates_warning_hold_abort() -> None:
    supervisor = TrackingSafetySupervisor(SafetyConfig())
    assert supervisor.evaluate(lost_tracking(), now_monotonic_s=10.0).status is SafetyStatus.WARNING
    assert (
        supervisor.evaluate(lost_tracking(), now_monotonic_s=10.3).status
        is SafetyStatus.HOLD_REQUIRED
    )
    aborted = supervisor.evaluate(lost_tracking(), now_monotonic_s=11.0)
    assert aborted.status is SafetyStatus.ABORT_REQUIRED
    assert aborted.reason is SafetyReason.TRACKING_TIMEOUT


def test_valid_tracking_resets_timeout() -> None:
    supervisor = TrackingSafetySupervisor(SafetyConfig())
    supervisor.evaluate(lost_tracking(), now_monotonic_s=10.0)
    valid = TrackingResult(10.2, True, 0.9, 0.5, 0.5, 0.1, 0.1, 0, 0)
    assert supervisor.evaluate(valid, now_monotonic_s=10.2).status is SafetyStatus.NORMAL
    assert supervisor.evaluate(lost_tracking(), now_monotonic_s=10.4).status is SafetyStatus.WARNING


def test_heartbeat_timeout_boundary() -> None:
    monitor = HeartbeatMonitor(2.0)
    assert monitor.evaluate(1.0).status is SafetyStatus.ABORT_REQUIRED
    monitor.record(10.0)
    assert monitor.evaluate(12.0).status is SafetyStatus.NORMAL
    assert monitor.evaluate(12.001).reason is SafetyReason.MAVLINK_HEARTBEAT_TIMEOUT

