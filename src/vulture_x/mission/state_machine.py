"""Explicit first-milestone state transitions through TRACK."""

from vulture_x.enums import MissionState


class InvalidTransitionError(ValueError):
    pass


_FORWARD_TRANSITIONS: dict[MissionState, frozenset[MissionState]] = {
    MissionState.BOOT: frozenset({MissionState.SELF_TEST}),
    MissionState.SELF_TEST: frozenset({MissionState.WAIT_FCU}),
    MissionState.WAIT_FCU: frozenset({MissionState.READY}),
    MissionState.READY: frozenset({MissionState.ARMED}),
    MissionState.ARMED: frozenset({MissionState.TAKEOFF}),
    MissionState.TAKEOFF: frozenset({MissionState.SEARCH}),
    MissionState.SEARCH: frozenset({MissionState.TRACK}),
    MissionState.TRACK: frozenset({MissionState.SEARCH}),
}

_ABORTABLE_STATES = frozenset(_FORWARD_TRANSITIONS)


class MissionStateMachine:
    def __init__(self) -> None:
        self._state = MissionState.BOOT
        self._history: list[MissionState] = [self._state]

    @property
    def state(self) -> MissionState:
        return self._state

    @property
    def history(self) -> tuple[MissionState, ...]:
        return tuple(self._history)

    def transition(
        self,
        target: MissionState,
        *,
        operator_reset: bool = False,
    ) -> None:
        if self._state is MissionState.ABORT:
            if target is not MissionState.SELF_TEST or not operator_reset:
                raise InvalidTransitionError("ABORT requires explicit operator reset to SELF_TEST")
        elif target is MissionState.ABORT and self._state in _ABORTABLE_STATES:
            pass
        elif target not in _FORWARD_TRANSITIONS.get(self._state, frozenset()):
            raise InvalidTransitionError(f"invalid transition: {self._state} -> {target}")
        self._state = target
        self._history.append(target)

