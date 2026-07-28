"""First-milestone mission supervision through visual TRACK."""

from vulture_x.enums import MissionState, SafetyStatus
from vulture_x.mission.state_machine import MissionStateMachine
from vulture_x.models import SafetyResult, TrackingResult


class MissionSupervisor:
    def __init__(self, state_machine: MissionStateMachine) -> None:
        self._state_machine = state_machine

    def observe_tracking(
        self,
        tracking: TrackingResult,
        safety: SafetyResult,
    ) -> MissionState:
        state = self._state_machine.state
        if safety.status is SafetyStatus.ABORT_REQUIRED:
            self._state_machine.transition(MissionState.ABORT)
        elif state is MissionState.SEARCH and tracking.detected:
            self._state_machine.transition(MissionState.TRACK)
        elif state is MissionState.TRACK and not tracking.detected:
            self._state_machine.transition(MissionState.SEARCH)
        return self._state_machine.state

