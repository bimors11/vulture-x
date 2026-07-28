"""Machine-readable transition guard results."""

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class GuardResult:
    allowed: bool
    reason: str


def require(condition: bool, reason: str) -> GuardResult:
    return GuardResult(condition, "ok" if condition else reason)

