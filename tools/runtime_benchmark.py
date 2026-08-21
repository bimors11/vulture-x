#!/usr/bin/env python3
"""Runtime capture benchmark/smoke helper.

This tool is non-flight: it does not open MAVLink and does not send commands.
It can measure the legacy file reader and report whether the appsink backend can
be constructed on the current machine.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from vulture_x.runtime.capture import (
    FileFrameSource,
    GstAppSinkFrameSource,
    appsink_pipeline_from_rtsp,
    appsink_pipeline_from_udp_h264,
)
from vulture_x.runtime.metrics import RuntimeMetrics


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--backend",
        choices=("file", "appsink-udp", "appsink-rtsp"),
        default="file",
    )
    parser.add_argument("--camera-dir", type=Path)
    parser.add_argument("--rtsp-url", default="")
    parser.add_argument("--duration-s", type=float, default=5.0)
    parser.add_argument("--udp-port", type=int, default=5600)
    parser.add_argument("--json", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    duration_s = max(0.1, float(args.duration_s))
    if args.backend == "file":
        if args.camera_dir is None:
            print("runtime_benchmark_status=failed reason=missing_camera_dir")
            return 2
        source = FileFrameSource(args.camera_dir)
    elif args.backend == "appsink-udp":
        source = GstAppSinkFrameSource(appsink_pipeline_from_udp_h264(port=args.udp_port))
    else:
        source = GstAppSinkFrameSource(appsink_pipeline_from_rtsp(args.rtsp_url))

    metrics = RuntimeMetrics()
    frames = 0
    started_ns = time.monotonic_ns()
    deadline = time.monotonic() + duration_s
    try:
        while time.monotonic() < deadline:
            read_started_ns = time.monotonic_ns()
            frame = source.read()
            metrics.observe("capture_ms", (time.monotonic_ns() - read_started_ns) / 1_000_000.0)
            if frame is None:
                metrics.increment("dropped_frames")
                time.sleep(0.002)
                continue
            frames += 1
            metrics.mark_rate("capture", frame.received_timestamp_ns)
    finally:
        source.close()
    elapsed_s = (time.monotonic_ns() - started_ns) / 1_000_000_000.0
    payload = {
        "backend": args.backend,
        "frames": frames,
        "elapsed_s": elapsed_s,
        "capture_fps": frames / elapsed_s if elapsed_s > 0 else 0.0,
        "metrics": metrics.summary(),
        "hardware": "current_machine",
    }
    if args.json:
        print(json.dumps(payload, sort_keys=True))
    else:
        print(
            "runtime_benchmark_status=complete "
            f"backend={payload['backend']} frames={frames} "
            f"elapsed_s={elapsed_s:.2f} capture_fps={payload['capture_fps']:.2f}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
