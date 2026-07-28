import asyncio

import pytest

from vulture_x.config import VehicleConfig
from vulture_x.models import GuidanceCommand
from vulture_x.vehicle.mavlink_client import MockMavlinkVehicle


def vehicle_config() -> VehicleConfig:
    return VehicleConfig(connection="udpin:0.0.0.0:14550")


def test_mock_requires_identity_and_explicit_arm_enable() -> None:
    async def scenario() -> None:
        vehicle = MockMavlinkVehicle(vehicle_config())
        await vehicle.connect()
        assert vehicle.validate_identity("ARDUPILOTMEGA", "QUADROTOR")
        with pytest.raises(PermissionError, match="explicit operator enable"):
            await vehicle.arm(operator_enabled=False)
        await vehicle.arm(operator_enabled=True)
        assert vehicle.armed

    asyncio.run(scenario())


def test_mock_rejects_expired_guidance() -> None:
    async def scenario() -> None:
        vehicle = MockMavlinkVehicle(vehicle_config())
        await vehicle.connect()
        vehicle.validate_identity("ARDUPILOTMEGA", "QUADROTOR")
        await vehicle.set_mode("GUIDED")
        command = GuidanceCommand(1.0, 1, 0, 0, 0, 1.25, "test")
        with pytest.raises(ValueError, match="expired"):
            await vehicle.send_guidance(command, now_monotonic_s=1.3)

    asyncio.run(scenario())

