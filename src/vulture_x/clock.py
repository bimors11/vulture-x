"""Injectable monotonic and UTC clocks."""

from datetime import UTC, datetime
from time import monotonic
from typing import Protocol


class Clock(Protocol):
    def monotonic(self) -> float:
        """Return monotonic seconds."""
        ...

    def utc_now(self) -> datetime:
        """Return a timezone-aware UTC timestamp."""
        ...


class SystemClock:
    def monotonic(self) -> float:
        return monotonic()

    def utc_now(self) -> datetime:
        return datetime.now(UTC)

