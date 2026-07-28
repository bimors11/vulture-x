"""Fail-closed recovery-mode selection."""

from vulture_x.enums import SafetyReason


def recovery_mode_for(reason: SafetyReason) -> str:
    if reason in {
        SafetyReason.MAVLINK_HEARTBEAT_TIMEOUT,
        SafetyReason.VEHICLE_NAVIGATION_UNHEALTHY,
        SafetyReason.MINIMUM_SEPARATION_VIOLATION,
    }:
        return "RTL"
    return "LOITER"

