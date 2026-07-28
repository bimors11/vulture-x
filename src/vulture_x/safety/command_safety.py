"""Final command expiry check immediately before transmission."""

from vulture_x.enums import SafetyReason, SafetyStatus
from vulture_x.models import GuidanceCommand, SafetyResult


def validate_command_expiry(
    command: GuidanceCommand,
    *,
    now_monotonic_s: float,
) -> SafetyResult:
    if not command.is_valid_at(now_monotonic_s):
        return SafetyResult(
            SafetyStatus.ABORT_REQUIRED,
            SafetyReason.COMMAND_EXPIRED,
            {"valid_until_monotonic_s": command.valid_until_monotonic_s},
        )
    return SafetyResult(SafetyStatus.NORMAL, SafetyReason.NONE, {})

