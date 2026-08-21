"""Small in-process runtime metrics collector."""

from __future__ import annotations

from dataclasses import dataclass
from statistics import fmean, median
from threading import Lock


@dataclass(frozen=True, slots=True)
class SampleSummary:
    mean: float
    p50: float
    p95: float
    p99: float
    max: float
    count: int


class RuntimeMetrics:
    def __init__(self, window: int = 240) -> None:
        self._window = window
        self._lock = Lock()
        self._samples: dict[str, list[float]] = {}
        self._counts: dict[str, int] = {}
        self._last_event_ns: dict[str, int] = {}

    def observe(self, name: str, value: float) -> None:
        with self._lock:
            values = self._samples.setdefault(name, [])
            values.append(value)
            if len(values) > self._window:
                del values[: len(values) - self._window]

    def increment(self, name: str, amount: int = 1) -> None:
        with self._lock:
            self._counts[name] = self._counts.get(name, 0) + amount

    def mark_rate(self, name: str, now_ns: int) -> None:
        with self._lock:
            previous_ns = self._last_event_ns.get(name)
            self._last_event_ns[name] = now_ns
        if previous_ns is None or now_ns <= previous_ns:
            return
        period_s = (now_ns - previous_ns) / 1_000_000_000.0
        if period_s > 0:
            self.observe(f"{name}_hz", 1.0 / period_s)

    def summary(self) -> dict[str, object]:
        with self._lock:
            samples = {name: tuple(values) for name, values in self._samples.items()}
            counts = dict(self._counts)
        payload: dict[str, object] = dict(counts)
        for name, values in samples.items():
            if not values:
                continue
            ordered = sorted(values)
            payload[name] = {
                "mean": fmean(values),
                "p50": median(values),
                "p95": percentile(ordered, 0.95),
                "p99": percentile(ordered, 0.99),
                "max": max(values),
                "count": len(values),
            }
        return payload


def percentile(ordered_values: list[float], fraction: float) -> float:
    if not ordered_values:
        raise ValueError("percentile requires at least one value")
    index = min(len(ordered_values) - 1, max(0, round((len(ordered_values) - 1) * fraction)))
    return ordered_values[index]
