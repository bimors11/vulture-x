"""CSV time-series telemetry logger."""

import csv
from collections.abc import Mapping
from pathlib import Path
from typing import TextIO


class TelemetryCsvLogger:
    def __init__(self, path: Path, fieldnames: tuple[str, ...]) -> None:
        if not fieldnames:
            raise ValueError("telemetry field names must not be empty")
        path.parent.mkdir(parents=True, exist_ok=True)
        self._stream: TextIO = path.open("w", encoding="utf-8", newline="")
        self._writer = csv.DictWriter(self._stream, fieldnames=fieldnames, extrasaction="raise")
        self._writer.writeheader()

    def write(self, values: Mapping[str, object]) -> None:
        self._writer.writerow(values)
        self._stream.flush()

    def close(self) -> None:
        self._stream.close()

    def __enter__(self) -> "TelemetryCsvLogger":
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()

