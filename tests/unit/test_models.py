from dataclasses import FrozenInstanceError

import pytest

from vulture_x.enums import MissionState
from vulture_x.models import GuidanceCommand


def test_mission_states_have_stable_machine_readable_values() -> None:
    assert MissionState.WAIT_FCU.value == "wait_fcu"
    assert MissionState.ABORT.value == "abort"


def test_guidance_command_expiration_boundary() -> None:
    command = GuidanceCommand(
        timestamp_monotonic_s=10.0,
        velocity_n_mps=1.0,
        velocity_e_mps=0.0,
        velocity_d_mps=0.0,
        yaw_deg=None,
        valid_until_monotonic_s=10.5,
        reason="unit-test",
    )
    assert command.is_valid_at(10.5)
    assert not command.is_valid_at(10.5001)


def test_domain_models_are_immutable() -> None:
    command = GuidanceCommand(10.0, 1.0, 0.0, 0.0, None, 10.5, "unit-test")
    with pytest.raises(FrozenInstanceError):
        command.velocity_n_mps = 2.0  # type: ignore[misc]

