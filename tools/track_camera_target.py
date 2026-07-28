#!/usr/bin/env python3
"""Non-commanding OpenCV diagnostic for the red Gazebo target."""

from __future__ import annotations

import argparse
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import cv2
import numpy as np

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
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--timeout-s", type=float, default=0.0)
    parser.add_argument("--min-area", type=float, default=80.0)
    return parser.parse_args()


def newest_image(camera_dir: Path) -> Path | None:
    candidates: list[tuple[float, Path]] = []
    for pattern in ("*.png", "*.jpg", "*.jpeg"):
        for path in camera_dir.glob(pattern):
            try:
                if path.is_file():
                    candidates.append((path.stat().st_mtime, path))
            except FileNotFoundError:
                continue
    if not candidates:
        return None
    return max(candidates, key=lambda item: item[0])[1]


def open_capture(pipeline: str) -> cv2.VideoCapture:
    capture = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not capture.isOpened():
        capture.release()
        capture = cv2.VideoCapture(pipeline)
    return capture


def start_gstreamer_bridge() -> tuple[subprocess.Popen[str], Path] | None:
    if shutil.which("gst-launch-1.0") is None:
        return None
    camera_dir = Path(tempfile.mkdtemp(prefix="vulture-x-target-camera-"))
    command = [
        "gst-launch-1.0",
        "-q",
        "udpsrc",
        "port=5600",
        "caps=application/x-rtp,media=video,clock-rate=90000,encoding-name=H264",
        "!",
        "rtph264depay",
        "!",
        "avdec_h264",
        "!",
        "videoconvert",
        "!",
        "jpegenc",
        "!",
        "multifilesink",
        f"location={camera_dir}/frame-%06d.jpg",
        "max-files=10",
    ]
    process = subprocess.Popen(
        command,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    return process, camera_dir


def detect_red_target(frame: np.ndarray, min_area: float) -> tuple[int, int, int, int] | None:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_red_a = np.array([0, 90, 80], dtype=np.uint8)
    upper_red_a = np.array([12, 255, 255], dtype=np.uint8)
    lower_red_b = np.array([168, 90, 80], dtype=np.uint8)
    upper_red_b = np.array([180, 255, 255], dtype=np.uint8)
    mask = cv2.bitwise_or(
        cv2.inRange(hsv, lower_red_a, upper_red_a),
        cv2.inRange(hsv, lower_red_b, upper_red_b),
    )
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) < min_area:
        return None
    return tuple(int(value) for value in cv2.boundingRect(contour))


def main() -> int:
    args = parse_args()
    running = True

    def stop(_signum: int, _frame: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    bridge: subprocess.Popen[str] | None = None
    camera_dir: Path | None = args.camera_dir
    capture = None if camera_dir else open_capture(args.pipeline)
    if capture is not None and not capture.isOpened():
        print("target_status=unavailable reason=opencv_capture_open_failed", flush=True)
        capture.release()
        bridge_result = start_gstreamer_bridge()
        if bridge_result is None:
            return 2
        bridge, camera_dir = bridge_result
        capture = None
        print(f"target_status=using_gstreamer_bridge dir={camera_dir}", flush=True)

    start = time.monotonic()
    last_report = 0.0
    frames = 0
    detections = 0

    while running:
        if args.timeout_s > 0 and time.monotonic() - start >= args.timeout_s:
            break

        if camera_dir:
            image_path = newest_image(camera_dir)
            frame = cv2.imread(str(image_path)) if image_path else None
            ok = frame is not None
        else:
            assert capture is not None
            ok, frame = capture.read()

        if not ok or frame is None:
            time.sleep(0.05)
            continue

        frames += 1
        bbox = detect_red_target(frame, args.min_area)
        now = time.monotonic()
        if bbox is not None:
            detections += 1
            x, y, width, height = bbox
            frame_height, frame_width = frame.shape[:2]
            cx = (x + width / 2) / frame_width
            cy = (y + height / 2) / frame_height
            if now - last_report >= 1.0:
                print(
                    "target_status=detected "
                    f"bbox={x},{y},{width},{height} center={cx:.3f},{cy:.3f}",
                    flush=True,
                )
                last_report = now
            cv2.rectangle(frame, (x, y), (x + width, y + height), (0, 255, 255), 2)
        elif now - last_report >= 1.0:
            print("target_status=not_detected", flush=True)
            last_report = now

        if not args.headless:
            cv2.imshow("Vulture-X Target Diagnostic", frame)
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                break

    if capture is not None:
        capture.release()
    if bridge is not None:
        bridge.terminate()
        try:
            bridge.wait(timeout=2)
        except subprocess.TimeoutExpired:
            bridge.kill()
    if not args.headless:
        cv2.destroyAllWindows()

    print(f"camera_frames={frames} target_detections={detections}", flush=True)
    return 0 if frames > 0 else 2


if __name__ == "__main__":
    sys.exit(main())
