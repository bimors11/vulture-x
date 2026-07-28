#!/usr/bin/env python3
"""Minimal OpenCV viewer for the Gazebo camera stream."""

from __future__ import annotations

import argparse
import os
import signal
import sys
import time
from pathlib import Path

import cv2

DEFAULT_PIPELINE = (
    "udpsrc port=5600 caps=application/x-rtp,media=video,clock-rate=90000,"
    "encoding-name=H264 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink sync=false"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--pipeline",
        default=os.environ.get("VULTURE_X_CAMERA_PIPELINE", DEFAULT_PIPELINE),
    )
    parser.add_argument("--camera-dir", type=Path, default=None)
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Read frames without opening a GUI window.",
    )
    parser.add_argument(
        "--timeout-s",
        type=float,
        default=0.0,
        help="Stop after this many seconds; 0 runs forever.",
    )
    return parser.parse_args()


def newest_image(camera_dir: Path) -> Path | None:
    images = sorted(
        (
            path
            for pattern in ("*.png", "*.jpg", "*.jpeg")
            for path in camera_dir.glob(pattern)
            if path.is_file()
        ),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )
    return images[0] if images else None


def open_capture(pipeline: str) -> cv2.VideoCapture:
    capture = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not capture.isOpened():
        capture.release()
        capture = cv2.VideoCapture(pipeline)
    return capture


def main() -> int:
    args = parse_args()
    running = True

    def stop(_signum: int, _frame: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    start = time.monotonic()
    frames = 0
    capture = None if args.camera_dir else open_capture(args.pipeline)
    if capture is not None and not capture.isOpened():
        print("camera_status=unavailable reason=opencv_capture_open_failed", flush=True)
        return 2

    while running:
        if args.timeout_s > 0 and time.monotonic() - start >= args.timeout_s:
            break

        if args.camera_dir:
            image_path = newest_image(args.camera_dir)
            frame = cv2.imread(str(image_path)) if image_path else None
            ok = frame is not None
        else:
            assert capture is not None
            ok, frame = capture.read()

        if not ok or frame is None:
            time.sleep(0.05)
            continue

        frames += 1
        if frames == 1:
            height, width = frame.shape[:2]
            print(f"camera_status=receiving width={width} height={height}", flush=True)

        if not args.headless:
            cv2.imshow("Vulture-X Gazebo Camera", frame)
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                break

    if capture is not None:
        capture.release()
    if not args.headless:
        cv2.destroyAllWindows()

    print(f"camera_frames={frames}", flush=True)
    return 0 if frames > 0 else 2


if __name__ == "__main__":
    sys.exit(main())
