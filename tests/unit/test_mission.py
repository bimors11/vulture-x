import pytest

from vulture_x.enums import MissionState
from vulture_x.mission.state_machine import InvalidTransitionError, MissionStateMachine


def test_valid_transitions_reach_track() -> None:
    machine = MissionStateMachine()
    for state in (
        MissionState.SELF_TEST,
        MissionState.WAIT_FCU,
        MissionState.READY,
        MissionState.ARMED,
        MissionState.TAKEOFF,
        MissionState.SEARCH,
        MissionState.TRACK,
    ):
        machine.transition(state)
    assert machine.state is MissionState.TRACK


def test_invalid_transition_is_rejected() -> None:
    machine = MissionStateMachine()
    with pytest.raises(InvalidTransitionError, match="invalid transition"):
        machine.transition(MissionState.TRACK)


def test_abort_from_active_state_requires_explicit_reset() -> None:
    machine = MissionStateMachine()
    machine.transition(MissionState.ABORT)
    with pytest.raises(InvalidTransitionError, match="explicit operator reset"):
        machine.transition(MissionState.SELF_TEST)
    machine.transition(MissionState.SELF_TEST, operator_reset=True)
    assert machine.state is MissionState.SELF_TEST

