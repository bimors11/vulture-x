"""Immutable domain models shared by Vulture-X modules."""

from dataclasses import dataclass

from vulture_x.enums import LimiterDecision, SafetyReason, SafetyStatus


@dataclass(frozen=True, slots=True)
class TrackingResult:
    timestamp_monotonic_s: float
    detected: bool
    confidence: float
    center_x_normalized: float
    center_y_normalized: float
    width_normalized: float
    height_normalized: float
    horizontal_error: float
    vertical_error: float


@dataclass(frozen=True, slots=True)
class GuidanceCommand:
    timestamp_monotonic_s: float
    velocity_forward_mps: float
    velocity_right_mps: float
    velocity_down_mps: float
    yaw_rate_deg_s: float
    valid_until_monotonic_s: float
    reason: str

    def is_valid_at(self, monotonic_s: float) -> bool:
        return monotonic_s <= self.valid_until_monotonic_s


@dataclass(frozen=True, slots=True)
class LimitedCommand:
    command: GuidanceCommand
    decision: LimiterDecision
    modified_fields: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class VehicleState:
    timestamp_monotonic_s: float
    connected: bool
    armed: bool
    mode: str
    latitude_deg: float | None
    longitude_deg: float | None
    altitude_msl_m: float | None
    local_north_m: float | None
    local_east_m: float | None
    local_down_m: float | None
    roll_deg: float | None
    pitch_deg: float | None
    yaw_deg: float | None
    battery_remaining_pct: float | None
    navigation_healthy: bool
    gps_healthy: bool


@dataclass(frozen=True, slots=True)
class SafetyResult:
    status: SafetyStatus
    reason: SafetyReason
    details: dict[str, object]

