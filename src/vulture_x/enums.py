"""Shared mission, safety, and command enumerations."""

from enum import StrEnum, auto


class MissionState(StrEnum):
    BOOT = auto()
    SELF_TEST = auto()
    WAIT_FCU = auto()
    READY = auto()
    ARMED = auto()
    TAKEOFF = auto()
    SEARCH = auto()
    TRACK = auto()
    GUIDANCE = auto()
    HOLD = auto()
    RETURN = auto()
    LAND = auto()
    ABORT = auto()
    COMPLETE = auto()


class SafetyStatus(StrEnum):
    NORMAL = auto()
    WARNING = auto()
    HOLD_REQUIRED = auto()
    ABORT_REQUIRED = auto()


class SafetyReason(StrEnum):
    NONE = auto()
    TARGET_NOT_DETECTED = auto()
    TRACKING_CONFIDENCE_LOW = auto()
    TRACKING_TIMEOUT = auto()
    MAVLINK_HEARTBEAT_TIMEOUT = auto()
    VEHICLE_NAVIGATION_UNHEALTHY = auto()
    COMMAND_EXPIRED = auto()
    COMMAND_LIMIT_EXCEEDED = auto()
    MINIMUM_SEPARATION_VIOLATION = auto()
    FCU_IDENTITY_INVALID = auto()
    COMMAND_ACK_FAILED = auto()
    OPERATOR_ABORT = auto()
    INTERNAL_ERROR = auto()


class LimiterDecision(StrEnum):
    ACCEPTED = auto()
    MODIFIED = auto()
    REJECTED = auto()
    ABORT_REQUIRED = auto()

