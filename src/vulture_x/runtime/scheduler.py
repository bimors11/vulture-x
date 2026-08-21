"""Fixed-deadline scheduling helpers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


class MonotonicClock(Protocol):
    def monotonic_ns(self) -> int:
        ...

    def sleep_ns(self, duration_ns: int) -> None:
        ...


@dataclass(slots=True)
class SchedulerTick:
    scheduled_ns: int
    started_ns: int
    period_ms: float | None
    jitter_ms: float | None
    deadline_missed: bool


class FixedDeadlineScheduler:
    def __init__(self, rate_hz: float, clock: MonotonicClock) -> None:
        if rate_hz <= 0:
            raise ValueError("rate_hz must be positive")
        self.period_ns = round(1_000_000_000 / rate_hz)
        self._clock = clock
        now_ns = self._clock.monotonic_ns()
        self._next_deadline_ns = now_ns
        self._last_started_ns: int | None = None

    def wait_next(self) -> SchedulerTick:
        now_ns = self._clock.monotonic_ns()
        if now_ns < self._next_deadline_ns:
            self._clock.sleep_ns(self._next_deadline_ns - now_ns)
        started_ns = self._clock.monotonic_ns()
        scheduled_ns = self._next_deadline_ns
        missed = started_ns > scheduled_ns + self.period_ns
        if missed:
            skipped_periods = max(1, (started_ns - scheduled_ns) // self.period_ns)
            self._next_deadline_ns = scheduled_ns + (skipped_periods + 1) * self.period_ns
        else:
            self._next_deadline_ns = scheduled_ns + self.period_ns
        period_ms = None
        jitter_ms = None
        if self._last_started_ns is not None:
            period_ms = (started_ns - self._last_started_ns) / 1_000_000.0
            jitter_ms = abs(period_ms - self.period_ns / 1_000_000.0)
        self._last_started_ns = started_ns
        return SchedulerTick(scheduled_ns, started_ns, period_ms, jitter_ms, missed)
