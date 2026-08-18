#!/usr/bin/env python3
"""Non-commanding OpenCV diagnostic for the Gazebo tracking banner."""

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

_HEAD_CASCADES: list[cv2.CascadeClassifier] | None = None

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


def detect_heads(
    frame: np.ndarray,
    *,
    max_width: int = 420,
    min_height_ratio: float = 0.05,
) -> list[tuple[int, int, int, int]]:
    """Detect faces/heads with OpenCV Haar cascades."""

    global _HEAD_CASCADES
    if _HEAD_CASCADES is None:
        cascade_names = (
            "haarcascade_frontalface_default.xml",
            "haarcascade_frontalface_alt2.xml",
            "haarcascade_profileface.xml",
        )
        _HEAD_CASCADES = [
            cv2.CascadeClassifier(str(Path(cv2.data.haarcascades) / name))
            for name in cascade_names
        ]
        _HEAD_CASCADES = [cascade for cascade in _HEAD_CASCADES if not cascade.empty()]

    frame_height, frame_width = frame.shape[:2]
    scale = 1.0
    detection_frame = frame
    if frame_width > max_width:
        scale = max_width / frame_width
        detection_frame = cv2.resize(
            frame,
            (max_width, max(1, round(frame_height * scale))),
            interpolation=cv2.INTER_AREA,
        )

    gray = cv2.cvtColor(detection_frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)
    candidates: list[tuple[float, tuple[int, int, int, int]]] = []
    for cascade in _HEAD_CASCADES:
        raw_rects = cascade.detectMultiScale(
            gray,
            scaleFactor=1.08,
            minNeighbors=4,
            minSize=(24, 24),
        )
        for rect in raw_rects:
            x, y, width, height = (int(value) for value in rect)
            if scale != 1.0:
                x = round(x / scale)
                y = round(y / scale)
                width = round(width / scale)
                height = round(height / scale)
            if width <= 0 or height <= 0:
                continue
            if height < frame_height * min_height_ratio:
                continue
            aspect = width / height
            if aspect < 0.55 or aspect > 1.45:
                continue
            pad_x = round(width * 0.18)
            pad_top = round(height * 0.28)
            pad_bottom = round(height * 0.14)
            head_bbox = (
                x - pad_x,
                y - pad_top,
                width + 2 * pad_x,
                height + pad_top + pad_bottom,
            )
            hx = max(0, min(frame_width - 1, head_bbox[0]))
            hy = max(0, min(frame_height - 1, head_bbox[1]))
            hwidth = max(1, min(frame_width - hx, head_bbox[2]))
            hheight = max(1, min(frame_height - hy, head_bbox[3]))
            area_score = float(width * height)
            candidates.append((area_score, (hx, hy, hwidth, hheight)))

    candidates.sort(key=lambda item: item[0], reverse=True)
    kept: list[tuple[int, int, int, int]] = []
    for _weight, bbox in candidates:
        if all(bbox_iou(bbox, previous) < 0.35 for previous in kept):
            kept.append(bbox)
    kept.sort(key=lambda item: (item[0], item[1]))
    return kept


def bbox_iou(
    first: tuple[int, int, int, int],
    second: tuple[int, int, int, int],
) -> float:
    ax, ay, aw, ah = first
    bx, by, bw, bh = second
    left = max(ax, bx)
    top = max(ay, by)
    right = min(ax + aw, bx + bw)
    bottom = min(ay + ah, by + bh)
    overlap = max(0, right - left) * max(0, bottom - top)
    union = aw * ah + bw * bh - overlap
    if union <= 0:
        return 0.0
    return overlap / union


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


def detect_banner_target(frame: np.ndarray, min_area: float) -> tuple[int, int, int, int] | None:
    """Detect the high-contrast magenta banner with a dark center mark.

    The plane world intentionally uses a large square-ish magenta banner instead
    of a small object. This detector prefers bright near-square components
    that contain a dark mark near their center, which avoids locking on runway
    markings or plain color patches.
    """

    color_mask = target_color_mask(frame)
    banner_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
    banner_mask = cv2.morphologyEx(
        banner_mask,
        cv2.MORPH_CLOSE,
        np.ones((19, 19), np.uint8),
    )
    marked_bbox = choose_target_candidate(frame, banner_mask, min_area, require_mark=True)
    if marked_bbox is not None:
        return marked_bbox

    blob_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
    blob_mask = cv2.morphologyEx(blob_mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    return choose_target_candidate(frame, blob_mask, min_area, require_mark=False)


def target_color_mask(frame: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hsv_mask = cv2.inRange(
        hsv,
        np.array([130, 45, 55], dtype=np.uint8),
        np.array([175, 255, 255], dtype=np.uint8),
    )
    bgr = frame.astype(np.int16)
    blue = bgr[:, :, 0]
    green = bgr[:, :, 1]
    red = bgr[:, :, 2]
    bgr_mask = (
        (red > 95)
        & (blue > 70)
        & (green < 150)
        & (red > green + 30)
        & (blue > green + 25)
    ).astype(np.uint8) * 255
    return cv2.bitwise_or(hsv_mask, bgr_mask)


def choose_target_candidate(
    frame: np.ndarray,
    mask: np.ndarray,
    min_area: float,
    *,
    require_mark: bool,
) -> tuple[int, int, int, int] | None:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    best_bbox: tuple[int, int, int, int] | None = None
    best_score = 0.0
    frame_area = float(frame.shape[0] * frame.shape[1])

    for contour in contours:
        contour_area = float(cv2.contourArea(contour))
        if contour_area < min_area:
            continue
        x, y, width, height = cv2.boundingRect(contour)
        if width <= 0 or height <= 0:
            continue
        touches_frame_edge = (
            x <= 2
            or y <= 2
            or x + width >= frame.shape[1] - 2
            or y + height >= frame.shape[0] - 2
        )
        if touches_frame_edge:
            continue

        bbox_area = float(width * height)
        if bbox_area > frame_area * 0.35:
            continue
        if width > frame.shape[1] * 0.80 or height > frame.shape[0] * 0.70:
            continue
        if not require_mark and (width > frame.shape[1] * 0.35 or height > frame.shape[0] * 0.35):
            continue
        aspect = width / height
        if aspect < 0.18 or aspect > 5.0:
            continue
        fill_ratio = contour_area / bbox_area
        if fill_ratio < 0.18:
            continue

        roi_gray = gray[y : y + height, x : x + width]
        dark_mask = cv2.inRange(roi_gray, 0, 85)
        dark_ratio = float(cv2.countNonZero(dark_mask)) / bbox_area
        center_roi = dark_mask[
            height // 3 : max(height // 3 + 1, 2 * height // 3),
            width // 3 : max(width // 3 + 1, 2 * width // 3),
        ]
        center_dark_ratio = (
            float(cv2.countNonZero(center_roi)) / float(center_roi.size)
            if center_roi.size
            else 0.0
        )
        if require_mark and (dark_ratio < 0.001 or center_dark_ratio < 0.003):
            continue
        if not require_mark and contour_area < max(min_area * 2.5, 80.0):
            continue

        squareness = 1.0 - min(1.0, abs(1.0 - aspect))
        score = contour_area * (1.0 + squareness + center_dark_ratio)
        if not require_mark:
            center_y = y + height / 2.0
            score *= 1.0 + 0.25 * (center_y / max(1.0, frame.shape[0]))
        if score > best_score:
            best_score = score
            pad = 2
            left = max(0, int(x) - pad)
            top = max(0, int(y) - pad)
            right = min(frame.shape[1], int(x + width) + pad)
            bottom = min(frame.shape[0], int(y + height) + pad)
            best_bbox = (left, top, right - left, bottom - top)

    return best_bbox


def detect_red_target(frame: np.ndarray, min_area: float) -> tuple[int, int, int, int] | None:
    """Compatibility wrapper for the current default banner-target mode."""

    return detect_banner_target(frame, min_area)


def detect_colored_target(frame: np.ndarray, min_area: float) -> tuple[int, int, int, int] | None:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_red_a = np.array([0, 90, 80], dtype=np.uint8)
    upper_red_a = np.array([14, 255, 255], dtype=np.uint8)
    lower_red_b = np.array([172, 90, 80], dtype=np.uint8)
    upper_red_b = np.array([180, 255, 255], dtype=np.uint8)
    mask = cv2.bitwise_or(
        cv2.inRange(hsv, lower_red_a, upper_red_a),
        cv2.inRange(hsv, lower_red_b, upper_red_b),
    )
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    frame_height, frame_width = frame.shape[:2]
    frame_area = float(frame_height * frame_width)
    best_bbox: tuple[int, int, int, int] | None = None
    best_score = 0.0
    for contour in contours:
        contour_area = float(cv2.contourArea(contour))
        if contour_area < min_area:
            continue
        x, y, width, height = cv2.boundingRect(contour)
        if width <= 0 or height <= 0:
            continue
        bbox_area = float(width * height)
        if bbox_area > frame_area * 0.18:
            continue
        aspect = width / height
        if aspect < 0.35 or aspect > 2.4:
            continue
        fill_ratio = contour_area / bbox_area
        if fill_ratio < 0.25:
            continue
        center_y_norm = (y + height / 2.0) / frame_height
        if center_y_norm > 0.82:
            continue
        score = contour_area * fill_ratio * (1.0 + min(width, height) / max(width, height))
        if score > best_score:
            best_score = score
            pad = 3
            left = max(0, int(x) - pad)
            top = max(0, int(y) - pad)
            right = min(frame_width, int(x + width) + pad)
            bottom = min(frame_height, int(y + height) + pad)
            best_bbox = (left, top, right - left, bottom - top)
    return best_bbox


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
