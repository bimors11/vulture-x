from dataclasses import FrozenInstanceError

import pytest

from vulture_x.enums import MissionState
from vulture_x.models import GuidanceCommand


def command(valid_until: float = 10.5) -> GuidanceCommand:
    return GuidanceCommand(10.0, 1.0, 0.0, 0.0, 0.0, valid_until, "unit-test")


def test_mission_states_have_stable_values() -> None:
    assert MissionState.WAIT_FCU.value == "wait_fcu"
    assert MissionState.GUIDANCE.value == "guidance"
    assert MissionState.ABORT.value == "abort"


def test_guidance_command_expiration_boundary() -> None:
    assert command().is_valid_at(10.5)
    assert not command().is_valid_at(10.5001)


def test_domain_models_are_immutable() -> None:
    value = command()
    with pytest.raises(FrozenInstanceError):
        value.velocity_forward_mps = 2.0  # type: ignore[misc]

