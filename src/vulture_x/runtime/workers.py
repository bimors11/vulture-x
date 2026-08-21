"""In-process runtime worker threads."""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass
from threading import Event, Lock, Thread

from vulture_x.runtime.buffers import LatestValue
from vulture_x.runtime.capture import FrameSource
from vulture_x.runtime.metrics import RuntimeMetrics
from vulture_x.runtime.scheduler import FixedDeadlineScheduler, MonotonicClock
from vulture_x.runtime.state import TrackingSnapshot, VehicleSnapshot, VideoFrame, WorkerHealth


class SystemClock:
    def monotonic_ns(self) -> int:
        return time.monotonic_ns()

    def sleep_ns(self, duration_ns: int) -> None:
        if duration_ns > 0:
            time.sleep(duration_ns / 1_000_000_000.0)


class WorkerBase:
    def __init__(self, name: str) -> None:
        self.name = name
        self._stop = Event()
        self._thread: Thread | None = None
        self._lock = Lock()
        self._heartbeat_ns: int | None = None
        self._fault: str | None = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = Thread(target=self._run_guarded, name=self.name, daemon=True)
        self._thread.start()

    def stop(self, timeout_s: float = 2.0) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout_s)

    def health(self) -> WorkerHealth:
        thread = self._thread
        with self._lock:
            return WorkerHealth(
                name=self.name,
                heartbeat_ns=self._heartbeat_ns,
                running=thread is not None and thread.is_alive(),
                fault=self._fault,
            )

    def _beat(self) -> None:
        with self._lock:
            self._heartbeat_ns = time.monotonic_ns()

    def _set_fault(self, exc: BaseException) -> None:
        with self._lock:
            self._fault = f"{type(exc).__name__}:{exc}"

    def _run_guarded(self) -> None:
        try:
            self.run()
        except Exception as exc:
            self._set_fault(exc)

    def run(self) -> None:
        raise NotImplementedError


class CaptureWorker(WorkerBase):
    def __init__(
        self,
        source: FrameSource,
        latest_frame: LatestValue[VideoFrame],
        metrics: RuntimeMetrics,
        *,
        idle_sleep_s: float = 0.002,
    ) -> None:
        super().__init__("capture")
        self._source = source
        self._latest_frame = latest_frame
        self._metrics = metrics
        self._idle_sleep_s = idle_sleep_s

    def run(self) -> None:
        try:
            while not self._stop.is_set():
                if not self.step_once():
                    time.sleep(self._idle_sleep_s)
        finally:
            self._source.close()

    def step_once(self) -> bool:
        start_ns = time.monotonic_ns()
        frame = self._source.read()
        self._metrics.observe("capture_ms", (time.monotonic_ns() - start_ns) / 1_000_000)
        self._beat()
        if frame is None:
            self._metrics.increment("dropped_frames")
            return False
        self._latest_frame.publish(frame)
        self._metrics.mark_rate("capture", frame.received_timestamp_ns)
        return True


class VisionWorker(WorkerBase):
    def __init__(
        self,
        latest_frame: LatestValue[VideoFrame],
        latest_tracking: LatestValue[TrackingSnapshot],
        metrics: RuntimeMetrics,
        processor: Callable[[VideoFrame], TrackingSnapshot | None],
        *,
        idle_sleep_s: float = 0.002,
    ) -> None:
        super().__init__("vision")
        self._latest_frame = latest_frame
        self._latest_tracking = latest_tracking
        self._metrics = metrics
        self._processor = processor
        self._idle_sleep_s = idle_sleep_s
        self.last_processed_sequence: int | None = None

    def run(self) -> None:
        while not self._stop.is_set():
            if self.step_once():
                continue
            time.sleep(self._idle_sleep_s)

    def step_once(self) -> bool:
        frame = self._latest_frame.get_latest()
        if frame is None or frame.sequence == self.last_processed_sequence:
            self._beat()
            return False
        start_ns = time.monotonic_ns()
        result = self._processor(frame)
        self._metrics.observe("vision_ms", (time.monotonic_ns() - start_ns) / 1_000_000)
        self._beat()
        self.last_processed_sequence = frame.sequence
        if result is None:
            self._metrics.increment("tracker_failures")
            return True
        self._latest_tracking.publish(result)
        self._metrics.mark_rate("vision", result.timestamp_ns)
        return True


class MavlinkRxWorker(WorkerBase):
    def __init__(
        self,
        latest_vehicle: LatestValue[VehicleSnapshot],
        metrics: RuntimeMetrics,
        receiver: Callable[[], VehicleSnapshot | None],
        *,
        idle_sleep_s: float = 0.005,
    ) -> None:
        super().__init__("mavlink_rx")
        self._latest_vehicle = latest_vehicle
        self._metrics = metrics
        self._receiver = receiver
        self._idle_sleep_s = idle_sleep_s

    def run(self) -> None:
        while not self._stop.is_set():
            if self.step_once():
                continue
            time.sleep(self._idle_sleep_s)

    def step_once(self) -> bool:
        snapshot = self._receiver()
        self._beat()
        if snapshot is None:
            return False
        self._latest_vehicle.publish(snapshot)
        self._metrics.mark_rate("mavlink_rx", snapshot.timestamp_ns)
        return True


@dataclass(frozen=True, slots=True)
class ControlInputs:
    now_ns: int
    frame: VideoFrame | None
    tracking: TrackingSnapshot | None
    vehicle: VehicleSnapshot | None
    frame_age_ms: float | None
    tracking_result_age_ms: float | None
    heartbeat_age_ms: float | None


class ControlWorker(WorkerBase):
    def __init__(
        self,
        latest_frame: LatestValue[VideoFrame],
        latest_tracking: LatestValue[TrackingSnapshot],
        latest_vehicle: LatestValue[VehicleSnapshot],
        metrics: RuntimeMetrics,
        controller: Callable[[ControlInputs], None],
        *,
        rate_hz: float = 30.0,
        clock: MonotonicClock | None = None,
    ) -> None:
        super().__init__("control")
        self._latest_frame = latest_frame
        self._latest_tracking = latest_tracking
        self._latest_vehicle = latest_vehicle
        self._metrics = metrics
        self._controller = controller
        self._clock = clock or SystemClock()
        self._scheduler = FixedDeadlineScheduler(rate_hz, self._clock)

    def run(self) -> None:
        while not self._stop.is_set():
            self.step_once()

    def step_once(self) -> None:
        tick = self._scheduler.wait_next()
        if tick.period_ms is not None:
            self._metrics.observe("control_period_ms", tick.period_ms)
        if tick.jitter_ms is not None:
            self._metrics.observe("control_jitter_ms", tick.jitter_ms)
        if tick.deadline_missed:
            self._metrics.increment("deadline_misses")
        start_ns = self._clock.monotonic_ns()
        inputs = control_inputs(
            start_ns,
            self._latest_frame.get_latest(),
            self._latest_tracking.get_latest(),
            self._latest_vehicle.get_latest(),
        )
        self._controller(inputs)
        self._metrics.mark_rate("control", start_ns)
        self._beat()


def control_inputs(
    now_ns: int,
    frame: VideoFrame | None,
    tracking: TrackingSnapshot | None,
    vehicle: VehicleSnapshot | None,
) -> ControlInputs:
    heartbeat_ns = vehicle.heartbeat_timestamp_ns if vehicle is not None else None
    return ControlInputs(
        now_ns=now_ns,
        frame=frame,
        tracking=tracking,
        vehicle=vehicle,
        frame_age_ms=age_ms(now_ns, frame.received_timestamp_ns if frame is not None else None),
        tracking_result_age_ms=age_ms(
            now_ns,
            tracking.timestamp_ns if tracking is not None else None,
        ),
        heartbeat_age_ms=age_ms(now_ns, heartbeat_ns),
    )


def age_ms(now_ns: int, timestamp_ns: int | None) -> float | None:
    if timestamp_ns is None:
        return None
    return max(0.0, (now_ns - timestamp_ns) / 1_000_000.0)
