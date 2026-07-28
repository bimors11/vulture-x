"""Mock-only MAVLink vehicle boundary for the first milestone."""

from time import monotonic

from vulture_x.config import VehicleConfig
from vulture_x.models import GuidanceCommand
from vulture_x.vehicle.commands import VehicleCommand, VehicleCommandType, velocity_command


class MockMavlinkVehicle:
    """Deterministic command recorder; it never communicates with real hardware."""

    def __init__(self, config: VehicleConfig) -> None:
        self._config = config
        self.connected = False
        self.identity_validated = False
        self.armed = False
        self.mode = "STANDBY"
        self.commands: list[VehicleCommand] = []

    async def connect(self) -> None:
        self.connected = True

    def validate_identity(self, autopilot: str, vehicle_type: str) -> bool:
        self.identity_validated = (
            autopilot == self._config.expected_autopilot
            and vehicle_type == self._config.expected_vehicle_type
        )
        return self.identity_validated

    async def set_mode(self, mode: str) -> None:
        self._require_identity()
        self.mode = mode
        self._record(VehicleCommandType.SET_MODE, {"mode": mode})

    async def arm(self, *, operator_enabled: bool) -> None:
        self._require_identity()
        if not operator_enabled:
            raise PermissionError("arming requires explicit operator enable")
        self.armed = True
        self._record(VehicleCommandType.ARM, {})

    async def send_guidance(
        self,
        command: GuidanceCommand,
        *,
        now_monotonic_s: float,
    ) -> None:
        self._require_identity()
        if self.mode != self._config.guided_mode:
            raise RuntimeError("guidance requires configured GUIDED mode")
        if not command.is_valid_at(now_monotonic_s):
            raise ValueError("expired guidance command rejected")
        self.commands.append(velocity_command(command))

    async def hold(self) -> None:
        await self.set_mode(self._config.hold_mode)
        self._record(VehicleCommandType.LOITER, {})

    async def return_to_launch(self) -> None:
        await self.set_mode(self._config.recovery_mode)
        self._record(VehicleCommandType.RTL, {})

    async def close(self) -> None:
        self.connected = False

    def _require_identity(self) -> None:
        if not self.connected or not self.identity_validated:
            raise RuntimeError("vehicle connection and FCU identity validation are required")

    def _record(self, command_type: VehicleCommandType, parameters: dict[str, object]) -> None:
        self.commands.append(VehicleCommand(command_type, monotonic(), parameters))

