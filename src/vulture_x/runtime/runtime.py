"""Coordinator for a single-process Vulture-X tracking runtime."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from vulture_x.runtime.buffers import LatestValue
from vulture_x.runtime.capture import FrameSource
from vulture_x.runtime.metrics import RuntimeMetrics
from vulture_x.runtime.state import RuntimeSnapshot, TrackingSnapshot, VehicleSnapshot, VideoFrame
from vulture_x.runtime.workers import (
    CaptureWorker,
    ControlInputs,
    ControlWorker,
    MavlinkRxWorker,
    VisionWorker,
    WorkerBase,
)


@dataclass
class TrackingRuntime:
    """Owns in-memory latest snapshots and worker lifecycle."""

    frame_source: FrameSource
    tracking_processor: Callable[[VideoFrame], TrackingSnapshot | None]
    vehicle_receiver: Callable[[], VehicleSnapshot | None]
    controller: Callable[[ControlInputs], None]
    control_rate_hz: float = 30.0

    def __post_init__(self) -> None:
        self.latest_frame: LatestValue[VideoFrame] = LatestValue()
        self.latest_tracking: LatestValue[TrackingSnapshot] = LatestValue()
        self.latest_vehicle: LatestValue[VehicleSnapshot] = LatestValue()
        self.metrics = RuntimeMetrics()
        self.capture_worker = CaptureWorker(self.frame_source, self.latest_frame, self.metrics)
        self.vision_worker = VisionWorker(
            self.latest_frame,
            self.latest_tracking,
            self.metrics,
            self.tracking_processor,
        )
        self.mavlink_rx_worker = MavlinkRxWorker(
            self.latest_vehicle,
            self.metrics,
            self.vehicle_receiver,
        )
        self.control_worker = ControlWorker(
            self.latest_frame,
            self.latest_tracking,
            self.latest_vehicle,
            self.metrics,
            self.controller,
            rate_hz=self.control_rate_hz,
        )
        self._workers: tuple[WorkerBase, ...] = (
            self.capture_worker,
            self.vision_worker,
            self.mavlink_rx_worker,
            self.control_worker,
        )

    def start(self) -> None:
        for worker in self._workers:
            worker.start()

    def stop(self) -> None:
        for worker in reversed(self._workers):
            worker.stop()

    def snapshot(self) -> RuntimeSnapshot:
        workers = {worker.name: worker.health() for worker in self._workers}
        return RuntimeSnapshot(
            frame=self.latest_frame.get_latest(),
            tracking=self.latest_tracking.get_latest(),
            vehicle=self.latest_vehicle.get_latest(),
            metrics=self.metrics.summary(),
            workers=workers,
        )
