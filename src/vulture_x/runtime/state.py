"""Immutable runtime snapshots shared by worker threads."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any

import numpy as np
from numpy.typing import NDArray


@dataclass(frozen=True, slots=True)
class VideoFrame:
    sequence: int
    image: NDArray[np.uint8]
    received_timestamp_ns: int
    source: str | None = None


@dataclass(frozen=True, slots=True)
class TrackingSnapshot:
    sequence: int
    frame_sequence: int
    timestamp_ns: int
    detected: bool
    bbox: tuple[int, int, int, int] | None = None
    confidence: float | None = None
    center_x: float | None = None
    center_y: float | None = None
    horizontal_error: float | None = None
    vertical_error: float | None = None
    mode: str = "unknown"
    vision_state: str = "ACQUIRE"
    details: Mapping[str, Any] = field(default_factory=lambda: MappingProxyType({}))


@dataclass(frozen=True, slots=True)
class VehicleSnapshot:
    timestamp_ns: int
    heartbeat_timestamp_ns: int | None
    mode: str | None
    armed: bool | None
    relative_altitude_m: float | None
    airspeed_mps: float | None
    heading_deg: float | None = None
    connected: bool = False
    details: Mapping[str, Any] = field(default_factory=lambda: MappingProxyType({}))


@dataclass(frozen=True, slots=True)
class WorkerHealth:
    name: str
    heartbeat_ns: int | None = None
    running: bool = False
    fault: str | None = None


@dataclass(frozen=True, slots=True)
class RuntimeSnapshot:
    frame: VideoFrame | None
    tracking: TrackingSnapshot | None
    vehicle: VehicleSnapshot | None
    metrics: Mapping[str, object]
    workers: Mapping[str, WorkerHealth]
