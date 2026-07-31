#!/usr/bin/env python3
"""SITL-only visual steering helper for the stationary Gazebo target."""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import time
from pathlib import Path

import cv2
from pymavlink import mavutil
from track_camera_target import detect_red_target, newest_image, open_capture

from vulture_x.vision.tracker import TemplateMatchingTracker

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
    parser.add_argument(
        "--vehicle",
        choices=("auto", "quad", "plane"),
        default="auto",
        help="Vehicle steering profile. auto uses the heartbeat MAV_TYPE.",
    )
    parser.add_argument("--forward-mps", type=float, default=3.0)
    parser.add_argument("--max-right-mps", type=float, default=1.2)
    parser.add_argument("--max-down-mps", type=float, default=3.0)
    parser.add_argument(
        "--vertical-gain",
        type=float,
        default=40.0,
        help="Image error to fixed-wing roll/pitch attitude gain.",
    )
    parser.add_argument(
        "--plane-vertical-lookahead-s",
        type=float,
        default=5.0,
        help="Fixed-wing altitude target lookahead for converting vertical speed into altitude.",
    )
    parser.add_argument(
        "--min-relative-alt-m",
        type=float,
        default=15.0,
        help="Minimum relative altitude for fixed-wing SITL steering.",
    )
    parser.add_argument(
        "--max-plane-pitch-deg",
        type=float,
        default=20.0,
        help="Fixed-wing pitch attitude limit for visual vertical correction.",
    )
    parser.add_argument(
        "--max-plane-roll-deg",
        type=float,
        default=35.0,
        help="Fixed-wing roll attitude limit for visual horizontal correction.",
    )
    parser.add_argument(
        "--plane-throttle",
        type=float,
        default=0.75,
        help="Fixed-wing throttle fraction while sending visual attitude corrections.",
    )
    parser.add_argument(
        "--plane-lead-s",
        type=float,
        default=0.35,
        help="Fixed-wing image-error lead time to reduce visual tracking undershoot.",
    )
    parser.add_argument("--max-yaw-rate-deg-s", type=float, default=35.0)
    parser.add_argument(
        "--tracking-mode",
        choices=("orange", "custom"),
        default="orange",
        help="orange detects the Gazebo marker; custom tracks a UI-selected ROI.",
    )
    parser.add_argument("--selection-file", type=Path, default=None)
    parser.add_argument("--tracker", choices=("CSRT", "KCF", "TEMPLATE"), default="TEMPLATE")
    parser.add_argument(
        "--enable-guidance",
        action="store_true",
        help="Required. Send bounded commands to local ArduPilot SITL.",
    )
    return parser.parse_args()


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def create_tracker(tracker_name: str) -> object:
    if tracker_name == "TEMPLATE":
        return TemplateMatchingTracker()
    direct_factory = getattr(cv2, f"Tracker{tracker_name}_create", None)
    if direct_factory is not None:
        return direct_factory()
    legacy = getattr(cv2, "legacy", None)
    legacy_factory = getattr(legacy, f"Tracker{tracker_name}_create", None)
    if legacy_factory is not None:
        return legacy_factory()
    raise RuntimeError(
        f"OpenCV {tracker_name} tracker is unavailable; install opencv-contrib-python"
    )


def selection_bbox(selection_file: Path, frame: object) -> tuple[int, int, int, int]:
    raw = json.loads(selection_file.read_text(encoding="utf-8"))
    if raw.get("mode") != "custom":
        raise RuntimeError("custom selection mode is not enabled")
    frame_height, frame_width = frame.shape[:2]
    x = float(raw["x"]) * frame_width
    y = float(raw["y"]) * frame_height
    width = float(raw["width"]) * frame_width
    height = float(raw["height"]) * frame_height
    bbox = (
        round(clamp(x, 0, frame_width - 1)),
        round(clamp(y, 0, frame_height - 1)),
        round(clamp(width, 1, frame_width)),
        round(clamp(height, 1, frame_height)),
    )
    if bbox[0] + bbox[2] > frame_width or bbox[1] + bbox[3] > frame_height:
        raise RuntimeError("custom selection is outside the camera frame")
    return bbox


class CustomSelectionTracker:
    def __init__(self, tracker_name: str, selection_file: Path) -> None:
        self._tracker_name = tracker_name
        self._selection_file = selection_file
        self._tracker: object | None = None
        self._selection_mtime_ns: int | None = None

    def bbox(self, frame: object) -> tuple[int, int, int, int] | None:
        if not self._selection_file.exists():
            raise RuntimeError("custom selection is missing; select a target in the UI first")
        mtime_ns = self._selection_file.stat().st_mtime_ns
        if self._tracker is None or mtime_ns != self._selection_mtime_ns:
            initial_bbox = selection_bbox(self._selection_file, frame)
            tracker = create_tracker(self._tracker_name)
            if tracker.init(frame, initial_bbox) is False:
                raise RuntimeError("OpenCV rejected the custom target selection")
            self._tracker = tracker
            self._selection_mtime_ns = mtime_ns
            return initial_bbox

        detected, bbox = self._tracker.update(frame)
        if not detected:
            return None
        return tuple(round(value) for value in bbox)


def wait_mode(connection: mavutil.mavfile, mode_name: str, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        message = connection.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if message is None:
            continue
        if mavutil.mode_string_v10(message) == mode_name:
            return True
    return False


def resolve_vehicle_profile(args: argparse.Namespace, heartbeat: object) -> str:
    if args.vehicle != "auto":
        return str(args.vehicle)
    if heartbeat.type == mavutil.mavlink.MAV_TYPE_QUADROTOR:
        return "quad"
    if heartbeat.type == mavutil.mavlink.MAV_TYPE_FIXED_WING:
        return "plane"
    raise RuntimeError(f"unexpected vehicle type: {heartbeat.type}")


def verify_connection(
    connection: mavutil.mavfile,
    args: argparse.Namespace,
) -> tuple[int, int, str]:
    timeout_s = float(args.timeout_s)
    heartbeat = connection.wait_heartbeat(timeout=timeout_s)
    if heartbeat is None:
        raise RuntimeError("no MAVLink heartbeat")
    if heartbeat.autopilot != mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
        raise RuntimeError(f"unexpected autopilot: {heartbeat.autopilot}")
    vehicle = resolve_vehicle_profile(args, heartbeat)
    if args.vehicle != "auto":
        expected_type = (
            mavutil.mavlink.MAV_TYPE_QUADROTOR
            if vehicle == "quad"
            else mavutil.mavlink.MAV_TYPE_FIXED_WING
        )
        if heartbeat.type != expected_type:
            raise RuntimeError(
                f"vehicle profile mismatch requested={vehicle} detected_type={heartbeat.type}"
            )

    mode = mavutil.mode_string_v10(heartbeat)
    armed = bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    if vehicle == "plane" and mode != "GUIDED":
        guided_mode = connection.mode_mapping().get("GUIDED")
        if guided_mode is None:
            raise RuntimeError("guided_mode_unavailable")
        print(
            f"sitl_tracking_status=setting_mode vehicle=plane from={mode} to=GUIDED",
            flush=True,
        )
        connection.set_mode(guided_mode)
        if not wait_mode(connection, "GUIDED", timeout_s):
            raise RuntimeError("guided_mode_timeout")
    elif mode != "GUIDED":
        raise RuntimeError(f"vehicle must already be in GUIDED mode, got {mode}")
    if not armed:
        raise RuntimeError("vehicle must already be armed; this script will not arm")

    return connection.target_system, connection.target_component, vehicle


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


def send_plane_speed(
    connection: mavutil.mavfile,
    target_system: int,
    target_component: int,
    airspeed_mps: float,
) -> None:
    connection.mav.command_int_send(
        target_system,
        target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_SPEED,
        0,
        0,
        mavutil.mavlink.SPEED_TYPE_AIRSPEED,
        airspeed_mps,
        0,
        0,
        0,
        0,
        0,
    )


def send_plane_altitude(
    connection: mavutil.mavfile,
    target_system: int,
    target_component: int,
    relative_altitude_m: float,
    vertical_speed_mps: float,
) -> None:
    connection.mav.command_int_send(
        target_system,
        target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_ALTITUDE,
        0,
        0,
        0,
        0,
        abs(vertical_speed_mps),
        0,
        0,
        0,
        relative_altitude_m,
    )


def send_plane_altitude_offset(
    connection: mavutil.mavfile,
    target_system: int,
    target_component: int,
    down_m: float,
) -> None:
    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )
    connection.mav.set_position_target_local_ned_send(
        int(time.monotonic() * 1000) & 0xFFFFFFFF,
        target_system,
        target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,
        type_mask,
        0,
        0,
        down_m,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )


def euler_to_quaternion(roll_rad: float, pitch_rad: float, yaw_rad: float) -> list[float]:
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def send_plane_attitude(
    connection: mavutil.mavfile,
    target_system: int,
    target_component: int,
    roll_deg: float,
    pitch_deg: float,
    throttle: float,
) -> None:
    # ArduPlane inverts this mask internally. Leaving roll-rate, pitch-rate,
    # and throttle bits clear marks them as the partial fields to use.
    type_mask = (
        mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE
        | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE
    )
    connection.mav.set_attitude_target_send(
        int(time.monotonic() * 1000) & 0xFFFFFFFF,
        target_system,
        target_component,
        type_mask,
        euler_to_quaternion(math.radians(roll_deg), math.radians(pitch_deg), 0.0),
        0.0,
        0.0,
        0.0,
        clamp(throttle, 0.0, 1.0),
    )


def send_plane_heading(
    connection: mavutil.mavfile,
    target_system: int,
    target_component: int,
    heading_deg: float,
    lateral_accel_mps2: float,
) -> None:
    connection.mav.command_int_send(
        target_system,
        target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_HEADING,
        0,
        0,
        mavutil.mavlink.HEADING_TYPE_HEADING,
        heading_deg % 360.0,
        max(0.05, lateral_accel_mps2),
        0,
        0,
        0,
        0,
    )


def read_plane_telemetry(
    connection: mavutil.mavfile,
    last_heading_deg: float,
    last_relative_alt_m: float,
) -> tuple[float, float]:
    deadline = time.monotonic() + 0.03
    heading_deg = last_heading_deg
    relative_alt_m = last_relative_alt_m
    while time.monotonic() < deadline:
        message = connection.recv_match(
            type=["VFR_HUD", "GLOBAL_POSITION_INT"],
            blocking=True,
            timeout=0.005,
        )
        if message is None:
            continue
        if message.get_type() == "VFR_HUD":
            heading_deg = float(message.heading)
        elif message.get_type() == "GLOBAL_POSITION_INT":
            relative_alt_m = float(message.relative_alt) / 1000.0
    return heading_deg, relative_alt_m


def plane_target_altitude(
    relative_altitude_m: float,
    down_mps: float,
    lookahead_s: float,
    min_relative_alt_m: float,
) -> float:
    target_altitude_m = relative_altitude_m - down_mps * max(0.1, lookahead_s)
    return max(min_relative_alt_m, target_altitude_m)


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
        target_system, target_component, vehicle = verify_connection(connection, args)
    except RuntimeError as exc:
        print(f"sitl_tracking_status=failed reason={exc}", flush=True)
        return 1

    capture = None if args.camera_dir else open_capture(args.pipeline or "")
    if capture is not None and not capture.isOpened():
        print("sitl_tracking_status=failed reason=camera_unavailable", flush=True)
        return 1

    custom_tracker = None
    if args.tracking_mode == "custom":
        if args.selection_file is None:
            print("sitl_tracking_status=failed reason=missing_custom_selection_file", flush=True)
            return 2
        custom_tracker = CustomSelectionTracker(args.tracker, args.selection_file)

    deadline = time.monotonic() + args.timeout_s
    period_s = 1.0 / args.rate_hz
    frames = 0
    detections = 0
    last_report = 0.0
    last_command = time.monotonic()
    plane_heading_deg = 0.0
    plane_relative_alt_m = 50.0
    previous_error_x: float | None = None
    previous_error_y: float | None = None
    previous_error_time: float | None = None
    if vehicle == "plane":
        plane_heading_deg, plane_relative_alt_m = read_plane_telemetry(
            connection,
            plane_heading_deg,
            plane_relative_alt_m,
        )
        send_plane_speed(connection, target_system, target_component, args.forward_mps)

    try:
        while running and time.monotonic() < deadline:
            ok, frame = frame_from_source(capture, args.camera_dir)
            if not ok or frame is None:
                time.sleep(0.05)
                continue

            frames += 1
            try:
                bbox = (
                    custom_tracker.bbox(frame)
                    if custom_tracker is not None
                    else detect_red_target(frame, args.min_area)
                )
            except RuntimeError as exc:
                print(f"sitl_tracking_status=failed reason={exc}", flush=True)
                return 1
            now = time.monotonic()
            if bbox is None:
                if vehicle == "quad":
                    send_body_velocity(connection, target_system, target_component, 0, 0, 0, 0)
                else:
                    previous_error_x = None
                    previous_error_y = None
                    previous_error_time = None
                    plane_heading_deg, plane_relative_alt_m = read_plane_telemetry(
                        connection,
                        plane_heading_deg,
                        plane_relative_alt_m,
                    )
                    send_plane_speed(connection, target_system, target_component, args.forward_mps)
                    send_plane_altitude(
                        connection,
                        target_system,
                        target_component,
                        plane_relative_alt_m,
                        args.max_down_mps,
                    )
                if now - last_report >= 1.0:
                    print(
                        f"sitl_tracking_status=searching mode={args.tracking_mode} "
                        "reason=target_not_detected",
                        flush=True,
                    )
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
            guided_error_x = error_x
            guided_error_y = error_y
            if (
                vehicle == "plane"
                and previous_error_x is not None
                and previous_error_y is not None
                and previous_error_time is not None
            ):
                dt = max(0.001, now - previous_error_time)
                lead_s = max(0.0, args.plane_lead_s)
                guided_error_x += ((error_x - previous_error_x) / dt) * lead_s
                guided_error_y += ((error_y - previous_error_y) / dt) * lead_s
            previous_error_x = error_x
            previous_error_y = error_y
            previous_error_time = now

            right_mps = clamp(error_x * 0.8, -args.max_right_mps, args.max_right_mps)
            down_mps = clamp(error_y * args.vertical_gain, -args.max_down_mps, args.max_down_mps)
            yaw_rate = clamp(
                error_x * args.max_yaw_rate_deg_s * 2.0,
                -args.max_yaw_rate_deg_s,
                args.max_yaw_rate_deg_s,
            )
            forward_mps = args.forward_mps if abs(error_x) < 0.25 else 0.0

            if vehicle == "quad":
                pitch_report = 0.0
                roll_report = 0.0
                send_body_velocity(
                    connection,
                    target_system,
                    target_component,
                    forward_mps,
                    right_mps,
                    down_mps,
                    yaw_rate,
                )
            else:
                plane_heading_deg, plane_relative_alt_m = read_plane_telemetry(
                    connection,
                    plane_heading_deg,
                    plane_relative_alt_m,
                )
                roll_deg = clamp(
                    guided_error_x * args.vertical_gain,
                    -args.max_plane_roll_deg,
                    args.max_plane_roll_deg,
                )
                pitch_deg = clamp(
                    -guided_error_y * args.vertical_gain,
                    -args.max_plane_pitch_deg,
                    args.max_plane_pitch_deg,
                )
                if plane_relative_alt_m <= args.min_relative_alt_m and down_mps > 0:
                    pitch_deg = max(0.0, pitch_deg)
                pitch_report = pitch_deg
                roll_report = roll_deg
                send_plane_speed(connection, target_system, target_component, args.forward_mps)
                send_plane_attitude(
                    connection,
                    target_system,
                    target_component,
                    roll_deg,
                    pitch_deg,
                    args.plane_throttle,
                )
            last_command = now

            if now - last_report >= 1.0:
                print(
                    "sitl_tracking_status=commanding "
                    f"vehicle={vehicle} "
                    f"mode={args.tracking_mode} "
                    f"center={center_x:.3f},{center_y:.3f} "
                    f"vel_body_frd={forward_mps:.2f},{right_mps:.2f},{down_mps:.2f} "
                    f"yaw_rate_deg_s={yaw_rate:.2f} "
                    f"roll_deg={roll_report:.2f} "
                    f"pitch_deg={pitch_report:.2f} "
                    f"guided_error={guided_error_x:.3f},{guided_error_y:.3f}",
                    flush=True,
                )
                last_report = now

            sleep_s = max(0.0, period_s - (time.monotonic() - last_command))
            time.sleep(sleep_s)
    finally:
        if vehicle == "quad":
            send_body_velocity(connection, target_system, target_component, 0, 0, 0, 0)
        else:
            send_plane_altitude(
                connection,
                target_system,
                target_component,
                plane_relative_alt_m,
                args.max_down_mps,
            )
        if capture is not None:
            capture.release()

    print(
        f"sitl_tracking_status=stopped camera_frames={frames} target_detections={detections}",
        flush=True,
    )
    return 0 if detections > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
