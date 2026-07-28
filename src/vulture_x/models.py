"""Immutable domain models shared by Vulture-X modules."""

from dataclasses import dataclass

from vulture_x.enums import SafetyReason, SafetyStatus


@dataclass(frozen=True, slots=True)
class TargetState:
    source_id: str
    timestamp_monotonic_s: float
    frame: str
    latitude_deg: float
    longitude_deg: float
    altitude_msl_m: float
    velocity_n_mps: float
    velocity_e_mps: float
    velocity_d_mps: float
    heading_deg: float | None
    position_accuracy_m: float | None
    velocity_accuracy_mps: float | None
    confidence: float


@dataclass(frozen=True, slots=True)
class VehicleState:
    timestamp_monotonic_s: float
    connected: bool
    armed: bool
    mode: str
    latitude_deg: float | None
    longitude_deg: float | None
    altitude_msl_m: float | None
    velocity_n_mps: float | None
    velocity_e_mps: float | None
    velocity_d_mps: float | None
    battery_remaining_pct: float | None
    ekf_healthy: bool
    gps_healthy: bool
    home_valid: bool


@dataclass(frozen=True, slots=True)
class RelativeState:
    timestamp_monotonic_s: float
    north_m: float
    east_m: float
    down_m: float
    velocity_n_mps: float
    velocity_e_mps: float
    velocity_d_mps: float
    horizontal_range_m: float
    vertical_separation_m: float
    range_3d_m: float
    closure_rate_mps: float
    confidence: float


@dataclass(frozen=True, slots=True)
class GuidanceCommand:
    timestamp_monotonic_s: float
    velocity_n_mps: float
    velocity_e_mps: float
    velocity_d_mps: float
    yaw_deg: float | None
    valid_until_monotonic_s: float
    reason: str

    def is_valid_at(self, monotonic_s: float) -> bool:
        """Return whether this command has not expired."""
        return monotonic_s <= self.valid_until_monotonic_s


@dataclass(frozen=True, slots=True)
class SafetyResult:
    status: SafetyStatus
    reason: SafetyReason
    details: dict[str, object]

