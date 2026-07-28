"""Shared enumerations for mission and safety behavior."""

from enum import StrEnum, auto


class MissionState(StrEnum):
    BOOT = auto()
    SELF_TEST = auto()
    WAIT_FCU = auto()
    WAIT_HOME = auto()
    READY = auto()
    ARMED_STANDBY = auto()
    TAKEOFF = auto()
    OBSERVE = auto()
    TRACK = auto()
    APPROACH = auto()
    STANDOFF = auto()
    RETURN = auto()
    LAND = auto()
    ABORT = auto()
    COMPLETE = auto()


class SafetyStatus(StrEnum):
    NORMAL = auto()
    WARNING = auto()
    RETURN_REQUIRED = auto()
    ABORT_REQUIRED = auto()


class SafetyReason(StrEnum):
    NONE = auto()
    FCU_HEARTBEAT_TIMEOUT = auto()
    TARGET_DATA_STALE = auto()
    TARGET_FRAME_INVALID = auto()
    VEHICLE_EKF_UNHEALTHY = auto()
    VEHICLE_POSITION_INVALID = auto()
    GEOFENCE_VIOLATION = auto()
    SEPARATION_VIOLATION = auto()
    BATTERY_LOW = auto()
    COMMAND_ACK_FAILED = auto()
    INTERNAL_ERROR = auto()
    OPERATOR_ABORT = auto()
    GUIDANCE_LIMIT_EXCEEDED = auto()
    ESTIMATOR_UNCERTAIN = auto()


class LimiterDecision(StrEnum):
    ACCEPTED = auto()
    MODIFIED = auto()
    REJECTED = auto()
    ABORT_REQUIRED = auto()

