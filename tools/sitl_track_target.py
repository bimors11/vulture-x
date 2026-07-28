#!/usr/bin/env python3
"""SITL-only visual steering helper for the stationary Gazebo target."""

from __future__ import annotations

import argparse
import os
import signal
import sys
import time
from pathlib import Path

import cv2
from pymavlink import mavutil
from track_camera_target import detect_red_target, newest_image, open_capture

DEFAULT_MAVLINK = "udpin:0.0.0.0:14550"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mavlink",
        default=os.environ.get("VULTURE_X_MAVLINK", DEFAULT_MAVLINK),
    )
    parser.add_argument("--camera-dir", type=Path, default=None)
    parser.add_argument(
        "--pipeline",
        default=os.environ.get("VULTURE_X_CAMERA_PIPELINE"),
    )
    parser.add_argument("--timeout-s", type=float, default=20.0)
    parser.add_argument("--min-area", type=float, default=80.0)
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--forward-mps", type=float, default=3.0)
    parser.add_argument("--max-right-mps", type=float, default=1.2)
    parser.add_argument("--max-down-mps", type=float, default=0.8)
    parser.add_argument("--max-yaw-rate-deg-s", type=float, default=35.0)
    parser.add_argument(
        "--enable-guidance",
        action="store_true",
        help="Required. Send bounded commands to local ArduPilot SITL.",
    )
    return parser.parse_args()


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def verify_connection(connection: mavutil.mavfile, timeout_s: float) -> tuple[int, int]:
    heartbeat = connection.wait_heartbeat(timeout=timeout_s)
    if heartbeat is None:
        raise RuntimeError("no MAVLink heartbeat")
    if heartbeat.autopilot != mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
        raise RuntimeError(f"unexpected autopilot: {heartbeat.autopilot}")
    if heartbeat.type != mavutil.mavlink.MAV_TYPE_QUADROTOR:
        raise RuntimeError(f"unexpected vehicle type: {heartbeat.type}")

    mode = mavutil.mode_string_v10(heartbeat)
    armed = bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    if mode != "GUIDED":
        raise RuntimeError(f"vehicle must already be in GUIDED mode, got {mode}")
    if not armed:
        raise RuntimeError("vehicle must already be armed; this script will not arm")

    return connection.target_system, connection.target_component


def send_body_velocity(
    connection: mavutil.mavfile,
    target_system: int,
    target_component: int,
    forward_mps: float,
    right_mps: float,
    down_mps: float,
    yaw_rate_deg_s: float,
) -> None:
    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    )
    connection.mav.set_position_target_local_ned_send(
        int(time.monotonic() * 1000) & 0xFFFFFFFF,
        target_system,
        target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        type_mask,
        0,
        0,
        0,
        forward_mps,
        right_mps,
        down_mps,
        0,
        0,
        0,
        0,
        yaw_rate_deg_s * 3.141592653589793 / 180.0,
    )


def frame_from_source(
    capture: cv2.VideoCapture | None,
    camera_dir: Path | None,
) -> tuple[bool, object | None]:
    if camera_dir is not None:
        image_path = newest_image(camera_dir)
        frame = cv2.imread(str(image_path)) if image_path else None
        return frame is not None, frame
    if capture is None:
        return False, None
    return capture.read()


def main() -> int:
    args = parse_args()
    if not args.enable_guidance:
        print("sitl_tracking_status=failed reason=missing_--enable-guidance", flush=True)
        return 2

    running = True

    def stop(_signum: int, _frame: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    connection = mavutil.mavlink_connection(
        args.mavlink,
        source_system=193,
        source_component=194,
        autoreconnect=False,
    )
    try:
        target_system, target_component = verify_connection(connection, 8.0)
    except RuntimeError as exc:
        print(f"sitl_tracking_status=failed reason={exc}", flush=True)
        return 1

    capture = None if args.camera_dir else open_capture(args.pipeline or "")
    if capture is not None and not capture.isOpened():
        print("sitl_tracking_status=failed reason=camera_unavailable", flush=True)
        return 1

    deadline = time.monotonic() + args.timeout_s
    period_s = 1.0 / args.rate_hz
    frames = 0
    detections = 0
    last_report = 0.0
    last_command = time.monotonic()

    try:
        while running and time.monotonic() < deadline:
            ok, frame = frame_from_source(capture, args.camera_dir)
            if not ok or frame is None:
                time.sleep(0.05)
                continue

            frames += 1
            bbox = detect_red_target(frame, args.min_area)
            now = time.monotonic()
            if bbox is None:
                send_body_velocity(connection, target_system, target_component, 0, 0, 0, 0)
                if now - last_report >= 1.0:
                    print("sitl_tracking_status=searching reason=target_not_detected", flush=True)
                    last_report = now
                time.sleep(period_s)
                continue

            detections += 1
            x, y, width, height = bbox
            frame_height, frame_width = frame.shape[:2]
            center_x = (x + width / 2) / frame_width
            center_y = (y + height / 2) / frame_height
            error_x = center_x - 0.5
            error_y = center_y - 0.5

            right_mps = clamp(error_x * 0.8, -args.max_right_mps, args.max_right_mps)
            down_mps = clamp(error_y * 0.6, -args.max_down_mps, args.max_down_mps)
            yaw_rate = clamp(
                error_x * args.max_yaw_rate_deg_s * 2.0,
                -args.max_yaw_rate_deg_s,
                args.max_yaw_rate_deg_s,
            )
            forward_mps = args.forward_mps if abs(error_x) < 0.25 else 0.0

            send_body_velocity(
                connection,
                target_system,
                target_component,
                forward_mps,
                right_mps,
                down_mps,
                yaw_rate,
            )
            last_command = now

            if now - last_report >= 1.0:
                print(
                    "sitl_tracking_status=commanding "
                    f"center={center_x:.3f},{center_y:.3f} "
                    f"vel_body_frd={forward_mps:.2f},{right_mps:.2f},{down_mps:.2f} "
                    f"yaw_rate_deg_s={yaw_rate:.2f}",
                    flush=True,
                )
                last_report = now

            sleep_s = max(0.0, period_s - (time.monotonic() - last_command))
            time.sleep(sleep_s)
    finally:
        send_body_velocity(connection, target_system, target_component, 0, 0, 0, 0)
        if capture is not None:
            capture.release()

    print(
        f"sitl_tracking_status=stopped camera_frames={frames} target_detections={detections}",
        flush=True,
    )
    return 0 if detections > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
