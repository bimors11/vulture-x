"""Typed high-level commands used by the mock vehicle boundary."""

from dataclasses import dataclass
from enum import StrEnum, auto

from vulture_x.models import GuidanceCommand


class VehicleCommandType(StrEnum):
    SET_MODE = auto()
    ARM = auto()
    TAKEOFF = auto()
    VELOCITY = auto()
    LOITER = auto()
    RTL = auto()
    LAND = auto()


@dataclass(frozen=True, slots=True)
class VehicleCommand:
    command_type: VehicleCommandType
    timestamp_monotonic_s: float
    parameters: dict[str, object]


def velocity_command(command: GuidanceCommand) -> VehicleCommand:
    return VehicleCommand(
        command_type=VehicleCommandType.VELOCITY,
        timestamp_monotonic_s=command.timestamp_monotonic_s,
        parameters={
            "velocity_forward_mps": command.velocity_forward_mps,
            "velocity_right_mps": command.velocity_right_mps,
            "velocity_down_mps": command.velocity_down_mps,
            "yaw_rate_deg_s": command.yaw_rate_deg_s,
            "valid_until_monotonic_s": command.valid_until_monotonic_s,
        },
    )

