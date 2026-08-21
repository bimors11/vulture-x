"""Thread-safe latest-value buffers.

The runtime intentionally drops superseded frames and telemetry snapshots. This
keeps slow workers from building stale FIFO backlogs.
"""

from __future__ import annotations

from threading import Lock
from typing import Generic, TypeVar

T = TypeVar("T")


class LatestValue(Generic[T]):
    """Store only the newest published value."""

    def __init__(self) -> None:
        self._lock = Lock()
        self._value: T | None = None
        self._revision = 0

    def publish(self, value: T) -> int:
        with self._lock:
            self._revision += 1
            self._value = value
            return self._revision

    def get_latest(self) -> T | None:
        with self._lock:
            return self._value

    def snapshot(self) -> tuple[int, T | None]:
        with self._lock:
            return self._revision, self._value
