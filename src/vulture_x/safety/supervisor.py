"""Composition point for safety results."""

from collections.abc import Iterable

from vulture_x.enums import SafetyStatus
from vulture_x.models import SafetyResult

_PRIORITY = {
    SafetyStatus.NORMAL: 0,
    SafetyStatus.WARNING: 1,
    SafetyStatus.HOLD_REQUIRED: 2,
    SafetyStatus.ABORT_REQUIRED: 3,
}


def highest_priority(results: Iterable[SafetyResult]) -> SafetyResult:
    values = tuple(results)
    if not values:
        raise ValueError("at least one safety result is required")
    return max(values, key=lambda result: _PRIORITY[result.status])

