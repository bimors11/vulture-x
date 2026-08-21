import os
import time
from pathlib import Path

import cv2
import numpy as np
import pytest

from vulture_x.runtime.buffers import LatestValue
from vulture_x.runtime.capture import (
    FileFrameSource,
    appsink_pipeline_from_rtsp,
    appsink_pipeline_from_udp_h264,
)
from vulture_x.runtime.mavlink import MavlinkTelemetryReceiver
from vulture_x.runtime.metrics import RuntimeMetrics
from vulture_x.runtime.preview import encode_runtime_preview_jpeg, runtime_preview_metadata
from vulture_x.runtime.runtime import TrackingRuntime
from vulture_x.runtime.scheduler import FixedDeadlineScheduler
from vulture_x.runtime.state import TrackingSnapshot, VehicleSnapshot, VideoFrame
from vulture_x.runtime.workers import (
    CaptureWorker,
    ControlInputs,
    ControlWorker,
    MavlinkRxWorker,
    VisionWorker,
    control_inputs,
)


class ManualClock:
    def __init__(self, now_ns: int = 0) -> None:
        self.now_ns = now_ns
        self.sleeps: list[int] = []

    def monotonic_ns(self) -> int:
        return self.now_ns

    def sleep_ns(self, duration_ns: int) -> None:
        self.sleeps.append(duration_ns)
        self.now_ns += duration_ns


class FakeSource:
    def __init__(self, frames: list[VideoFrame | None]) -> None:
        self.frames = frames
        self.closed = False

    def read(self) -> VideoFrame | None:
        if not self.frames:
            return None
        return self.frames.pop(0)

    def close(self) -> None:
        self.closed = True


def frame(sequence: int, timestamp_ns: int | None = None) -> VideoFrame:
    image = np.zeros((4, 6, 3), dtype=np.uint8)
    return VideoFrame(sequence, image, timestamp_ns or sequence * 1_000_000, "test")


def tracking(sequence: int, frame_sequence: int, timestamp_ns: int) -> TrackingSnapshot:
    return TrackingSnapshot(
        sequence=sequence,
        frame_sequence=frame_sequence,
        timestamp_ns=timestamp_ns,
        detected=True,
        bbox=(1, 1, 2, 2),
        confidence=0.8,
        center_x=0.5,
        center_y=0.5,
        horizontal_error=0.0,
        vertical_error=0.0,
        mode="custom",
        vision_state="TRACK",
    )


def vehicle(timestamp_ns: int, heartbeat_ns: int | None = None) -> VehicleSnapshot:
    return VehicleSnapshot(
        timestamp_ns=timestamp_ns,
        heartbeat_timestamp_ns=heartbeat_ns if heartbeat_ns is not None else timestamp_ns,
        mode="FBWA",
        armed=True,
        relative_altitude_m=50.0,
        airspeed_mps=20.0,
        connected=True,
    )


class FakeMavlinkMessage:
    def __init__(self, message_type: str, **fields: object) -> None:
        self._message_type = message_type
        for key, value in fields.items():
            setattr(self, key, value)

    def get_type(self) -> str:
        return self._message_type


class FakeMavlinkConnection:
    def __init__(self, messages: list[FakeMavlinkMessage]) -> None:
        self.messages = messages

    def recv_match(
        self,
        *,
        type: list[str],
        blocking: bool,
        timeout: float,
    ) -> FakeMavlinkMessage | None:
        del type, blocking, timeout
        if not self.messages:
            return None
        return self.messages.pop(0)


def test_latest_value_overwrites_intermediate_values() -> None:
    latest: LatestValue[int] = LatestValue()

    assert latest.get_latest() is None
    assert latest.publish(101) == 1
    latest.publish(102)
    latest.publish(103)

    revision, value = latest.snapshot()
    assert revision == 3
    assert value == 103
    assert latest.get_latest() == 103


def test_file_frame_source_reads_legacy_latest_jpeg(tmp_path: Path) -> None:
    image = np.full((8, 10, 3), 127, dtype=np.uint8)
    image_path = tmp_path / "frame-000001.jpg"
    assert cv2.imwrite(str(image_path), image)
    old = time.time() - 1.0
    os.utime(image_path, (old, old))

    source = FileFrameSource(tmp_path)
    captured = source.read()

    assert captured is not None
    assert captured.sequence == 1
    assert captured.image.shape == image.shape
    assert source.read() is None


def test_appsink_pipeline_uses_latest_only_buffering() -> None:
    udp_pipeline = appsink_pipeline_from_udp_h264()
    rtsp_pipeline = appsink_pipeline_from_rtsp("rtsp://example.invalid/stream")

    assert "appsink name=sink" in udp_pipeline
    assert "max-buffers=1" in udp_pipeline
    assert "drop=true" in udp_pipeline
    assert "sync=false" in udp_pipeline
    assert "appsink name=sink" in rtsp_pipeline
    with pytest.raises(ValueError, match="rtsp_url_required"):
        appsink_pipeline_from_rtsp("http://example.invalid/stream")


def test_capture_worker_publishes_latest_frame_and_closes_source() -> None:
    latest: LatestValue[VideoFrame] = LatestValue()
    metrics = RuntimeMetrics()
    source = FakeSource([frame(1)])
    worker = CaptureWorker(source, latest, metrics)

    assert worker.step_once() is True
    worker.stop()
    source.close()

    assert latest.get_latest().sequence == 1  # type: ignore[union-attr]
    assert worker.health().heartbeat_ns is not None


def test_vision_worker_skips_obsolete_intermediate_frames() -> None:
    latest_frame: LatestValue[VideoFrame] = LatestValue()
    latest_tracking: LatestValue[TrackingSnapshot] = LatestValue()
    processed: list[int] = []

    def process(value: VideoFrame) -> TrackingSnapshot:
        processed.append(value.sequence)
        return tracking(len(processed), value.sequence, value.received_timestamp_ns + 10)

    worker = VisionWorker(latest_frame, latest_tracking, RuntimeMetrics(), process)
    latest_frame.publish(frame(101))
    latest_frame.publish(frame(102))
    latest_frame.publish(frame(103))
    latest_frame.publish(frame(104))

    assert worker.step_once() is True
    assert processed == [104]
    assert latest_tracking.get_latest().frame_sequence == 104  # type: ignore[union-attr]
    assert worker.step_once() is False


def test_mavlink_rx_worker_publishes_vehicle_snapshot() -> None:
    latest: LatestValue[VehicleSnapshot] = LatestValue()
    values = [vehicle(1_000)]

    def receive() -> VehicleSnapshot | None:
        return values.pop(0) if values else None

    worker = MavlinkRxWorker(latest, RuntimeMetrics(), receive)

    assert worker.step_once() is True
    assert latest.get_latest().mode == "FBWA"  # type: ignore[union-attr]
    assert worker.step_once() is False


def test_mavlink_telemetry_receiver_preserves_unknowns_until_seen() -> None:
    connection = FakeMavlinkConnection(
        [
            FakeMavlinkMessage("GLOBAL_POSITION_INT", relative_alt=32100),
            FakeMavlinkMessage("VFR_HUD", heading=91, airspeed=18.5),
        ]
    )
    receiver = MavlinkTelemetryReceiver(connection)  # type: ignore[arg-type]

    snapshot = receiver.receive()

    assert snapshot is not None
    assert snapshot.mode is None
    assert snapshot.armed is None
    assert snapshot.relative_altitude_m == pytest.approx(32.1)
    assert snapshot.airspeed_mps == pytest.approx(18.5)


def test_fixed_deadline_scheduler_does_not_accumulate_backlog() -> None:
    clock = ManualClock(0)
    scheduler = FixedDeadlineScheduler(30.0, clock)

    first = scheduler.wait_next()
    clock.now_ns += 100_000_000
    second = scheduler.wait_next()
    third = scheduler.wait_next()

    assert first.deadline_missed is False
    assert second.deadline_missed is True
    assert third.scheduled_ns > second.scheduled_ns
    assert clock.sleeps[-1] > 0


def test_control_worker_reports_freshness_ages_and_period_metrics() -> None:
    frames: LatestValue[VideoFrame] = LatestValue()
    tracks: LatestValue[TrackingSnapshot] = LatestValue()
    vehicles: LatestValue[VehicleSnapshot] = LatestValue()
    frames.publish(frame(1, 1_000_000))
    tracks.publish(tracking(1, 1, 2_000_000))
    vehicles.publish(vehicle(3_000_000, heartbeat_ns=4_000_000))
    calls: list[ControlInputs] = []
    clock = ManualClock(10_000_000)
    worker = ControlWorker(
        frames,
        tracks,
        vehicles,
        RuntimeMetrics(),
        calls.append,
        rate_hz=30.0,
        clock=clock,
    )

    worker.step_once()
    clock.now_ns += 33_333_333
    worker.step_once()

    assert calls[-1].frame_age_ms == pytest.approx(42.333333)
    assert calls[-1].tracking_result_age_ms == pytest.approx(41.333333)
    assert calls[-1].heartbeat_age_ms == pytest.approx(39.333333)
    assert worker.health().heartbeat_ns is not None


def test_control_inputs_keeps_unknown_vehicle_values_none() -> None:
    inputs = control_inputs(10_000_000, None, None, None)

    assert inputs.frame_age_ms is None
    assert inputs.tracking_result_age_ms is None
    assert inputs.heartbeat_age_ms is None


def test_tracking_runtime_starts_stops_and_exposes_snapshot() -> None:
    source = FakeSource([frame(1)])

    def process(value: VideoFrame) -> TrackingSnapshot:
        return tracking(1, value.sequence, value.received_timestamp_ns + 1)

    def receive() -> VehicleSnapshot:
        return vehicle(2_000_000)

    controllers: list[ControlInputs] = []
    runtime = TrackingRuntime(source, process, receive, controllers.append)
    runtime.capture_worker.step_once()
    runtime.vision_worker.step_once()
    runtime.mavlink_rx_worker.step_once()
    snapshot = runtime.snapshot()
    runtime.stop()

    assert snapshot.frame is not None
    assert snapshot.tracking is not None
    assert snapshot.vehicle is not None
    assert set(snapshot.workers) == {"capture", "vision", "mavlink_rx", "control"}


def test_runtime_preview_encodes_native_frame_once() -> None:
    source_frame = frame(7)
    runtime_snapshot = TrackingRuntime(
        FakeSource([]),
        lambda value: tracking(1, value.sequence, value.received_timestamp_ns),
        lambda: None,
        lambda _inputs: None,
    ).snapshot()

    calls: list[tuple[int, int]] = []

    def render(image: object, _snapshot: object) -> None:
        calls.append(image.shape[:2])  # type: ignore[attr-defined]

    encoded = encode_runtime_preview_jpeg(source_frame, runtime_snapshot, hud_renderer=render)
    assert encoded is not None
    decoded = cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8), cv2.IMREAD_COLOR)
    metadata = runtime_preview_metadata(runtime_snapshot)

    assert calls == [(4, 6)]
    assert decoded.shape == source_frame.image.shape
    assert metadata["frame_sequence"] is None


def test_worker_fault_is_reported() -> None:
    latest_frame: LatestValue[VideoFrame] = LatestValue()
    latest_tracking: LatestValue[TrackingSnapshot] = LatestValue()
    latest_frame.publish(frame(1))

    def fail(_value: VideoFrame) -> TrackingSnapshot:
        raise RuntimeError("boom")

    worker = VisionWorker(latest_frame, latest_tracking, RuntimeMetrics(), fail)
    worker._run_guarded()

    assert worker.health().fault == "RuntimeError:boom"
