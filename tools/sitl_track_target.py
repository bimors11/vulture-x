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
from typing import NamedTuple

import cv2
from mavlink_endpoint import open_mavlink_connection
from pymavlink import mavutil
from track_camera_target import detect_red_target, newest_image, open_capture

from vulture_x.vision.tracker import TemplateMatchingTracker, expand_bbox

DEFAULT_MAVLINK = "udpin:0.0.0.0:14550"
PLANE_TARGET_AIRSPEED_MPS = 20.0
PLANE_CRUISE_THROTTLE = 0.55
PLANE_RESPONSE_PARAM_NAMES = (
    "LIM_ROLL_CD",
    "LIM_PITCH_MAX",
    "LIM_PITCH_MIN",
    "TRIM_THROTTLE",
    "THR_MIN",
    "THR_MAX",
    "ARSPD_FBW_MIN",
    "ARSPD_FBW_MAX",
    "RLL2SRV_TCONST",
    "PTCH2SRV_TCONST",
    "RLL2SRV_P",
    "RLL2SRV_I",
    "RLL2SRV_D",
    "PTCH2SRV_P",
    "PTCH2SRV_I",
    "PTCH2SRV_D",
    "NAVL1_PERIOD",
    "NAVL1_DAMPING",
    "RCMAP_PITCH",
    "RC2_REVERSED",
)


class PlaneResponseModel(NamedTuple):
    max_roll_deg: float
    max_pitch_up_deg: float
    max_pitch_down_deg: float
    cruise_throttle: float
    min_throttle: float
    max_throttle: float
    target_airspeed_mps: float
    pitch_filter_alpha: float
    max_pitch_step_deg: float
    pitch_rc_reversed: bool
    raw_params: dict[str, float]


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
    parser.add_argument("--min-area", type=float, default=25.0)
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
        default=52.0,
        help="Image error to fixed-wing roll/pitch attitude gain.",
    )
    parser.add_argument(
        "--plane-centering-gain",
        type=float,
        default=1.15,
        help="Extra fixed-wing roll/pitch gain for keeping the target on the crosshair.",
    )
    parser.add_argument(
        "--plane-near-centering-gain",
        type=float,
        default=2.15,
        help="Extra fixed-wing centering gain as bbox proximity approaches near-target.",
    )
    parser.add_argument(
        "--plane-damping-gain",
        type=float,
        default=0.22,
        help="Fixed-wing damping gain against image-error rate while target is far.",
    )
    parser.add_argument(
        "--plane-near-damping-gain",
        type=float,
        default=0.45,
        help="Fixed-wing damping gain against image-error rate near target.",
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
        default=40.0,
        help="Fixed-wing pitch attitude limit for visual vertical correction.",
    )
    parser.add_argument(
        "--max-plane-roll-deg",
        type=float,
        default=45.0,
        help="Fixed-wing roll attitude limit for visual horizontal correction.",
    )
    parser.add_argument(
        "--plane-roll-gain-scale",
        type=float,
        default=1.35,
        help="Internal fixed-wing roll gain multiplier for camera frame centering.",
    )
    parser.add_argument(
        "--plane-pitch-gain-scale",
        type=float,
        default=1.10,
        help="Fixed-wing pitch gain multiplier when the target appears far away.",
    )
    parser.add_argument(
        "--plane-pitch-near-gain-scale",
        type=float,
        default=1.45,
        help="Fixed-wing pitch gain multiplier when bbox size indicates a near target.",
    )
    parser.add_argument(
        "--plane-far-control-scale",
        type=float,
        default=0.55,
        help="Minimum fixed-wing roll/pitch control scale while target appears far away.",
    )
    parser.add_argument(
        "--plane-near-control-scale",
        type=float,
        default=1.0,
        help="Minimum roll/pitch control scale as bbox proximity approaches near-target.",
    )
    parser.add_argument(
        "--plane-near-pitch-down-limit-deg",
        type=float,
        default=40.0,
        help="Fixed-wing nose-down pitch limit when the target appears very near.",
    )
    parser.add_argument(
        "--plane-pitch-below-center-boost",
        type=float,
        default=0.25,
        help="Extra fixed-wing pitch-down gain when the target is below frame center.",
    )
    parser.add_argument(
        "--plane-pitch-filter-alpha",
        type=float,
        default=0.25,
        help="Low-pass filter alpha for fixed-wing pitch commands.",
    )
    parser.add_argument(
        "--plane-max-pitch-step-deg",
        type=float,
        default=2.0,
        help="Maximum fixed-wing pitch command change per camera update.",
    )
    parser.add_argument(
        "--plane-max-roll-step-deg",
        type=float,
        default=3.0,
        help="Maximum fixed-wing roll command change per camera update.",
    )
    parser.add_argument(
        "--plane-loss-hold-s",
        type=float,
        default=1.5,
        help="Bounded time to keep the last fixed-wing attitude command after target loss.",
    )
    parser.add_argument(
        "--plane-camera-hfov-deg",
        type=float,
        default=70.0,
        help="Fixed-wing camera horizontal FOV for perspective-corrected image error.",
    )
    parser.add_argument(
        "--plane-throttle",
        type=float,
        default=PLANE_CRUISE_THROTTLE,
        help="Fixed-wing cruise throttle fallback while airspeed telemetry is unavailable.",
    )
    parser.add_argument(
        "--plane-airspeed-mps",
        type=float,
        default=PLANE_TARGET_AIRSPEED_MPS,
        help="Fixed-wing target airspeed for FBWA throttle governor.",
    )
    parser.add_argument(
        "--plane-throttle-airspeed-gain",
        type=float,
        default=0.04,
        help="Throttle fraction adjustment per m/s of airspeed error.",
    )
    parser.add_argument(
        "--plane-min-throttle",
        type=float,
        default=0.25,
        help="Minimum fixed-wing throttle fraction when airspeed is high or nose-down.",
    )
    parser.add_argument(
        "--plane-max-throttle",
        type=float,
        default=0.80,
        help="Fixed-wing throttle fraction at maximum commanded nose-up pitch.",
    )
    parser.add_argument(
        "--plane-near-throttle-reduction",
        type=float,
        default=0.0,
        help="Throttle fraction reduction as bbox proximity approaches near-target.",
    )
    parser.add_argument(
        "--read-plane-params",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Read ArduPlane response/limit parameters before steering.",
    )
    parser.add_argument(
        "--plane-param-timeout-s",
        type=float,
        default=0.12,
        help="Per-parameter timeout for read-only plane parameter requests.",
    )
    parser.add_argument(
        "--plane-lead-s",
        type=float,
        default=0.05,
        help="Fixed-wing image-error lead time to reduce visual tracking undershoot.",
    )
    parser.add_argument(
        "--plane-error-deadband",
        type=float,
        default=0.015,
        help="Fixed-wing normalized image-error deadband before roll/pitch correction.",
    )
    parser.add_argument(
        "--source-system",
        type=int,
        default=int(os.environ.get("VULTURE_X_MAVLINK_SOURCE_SYSTEM", "255")),
        help="MAVLink source system id. Plane RC override needs the configured GCS sysid.",
    )
    parser.add_argument(
        "--source-component",
        type=int,
        default=int(os.environ.get("VULTURE_X_MAVLINK_SOURCE_COMPONENT", "194")),
    )
    parser.add_argument("--max-yaw-rate-deg-s", type=float, default=35.0)
    parser.add_argument(
        "--tracking-mode",
        choices=("banner", "orange", "custom"),
        default="banner",
        help=(
            "banner detects the Gazebo tracking banner; orange is a legacy alias; "
            "custom tracks a UI-selected ROI."
        ),
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


def perspective_correct_error(
    error: float,
    fov_deg: float,
) -> float:
    half_fov_rad = math.radians(clamp(fov_deg, 5.0, 170.0)) * 0.5
    edge_ray = math.tan(half_fov_rad)
    angular_error = math.atan(clamp(error * 2.0, -1.0, 1.0) * edge_ray)
    return 0.5 * angular_error / half_fov_rad


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
        self._smoothed_bbox: tuple[int, int, int, int] | None = None
        self._misses = 0
        self._max_held_misses = 4

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
            self._smoothed_bbox = initial_bbox
            self._misses = 0
            return initial_bbox

        detected, bbox = self._tracker.update(frame)
        frame_height, frame_width = frame.shape[:2]
        if not detected:
            self._misses += 1
            if self._smoothed_bbox is not None and self._misses <= self._max_held_misses:
                return self._smoothed_bbox
            self._tracker = None
            return None

        rounded_bbox = tuple(round(value) for value in bbox)
        if self._smoothed_bbox is not None and not bbox_transition_plausible(
            self._smoothed_bbox,
            rounded_bbox,
            frame_width,
            frame_height,
            max_center_jump_norm=0.08,
            max_size_ratio=2.0,
        ):
            self._misses += 1
            if self._misses <= self._max_held_misses:
                return self._smoothed_bbox
            self._tracker = None
            return None

        self._misses = 0
        self._smoothed_bbox = stabilize_bbox(
            self._smoothed_bbox,
            rounded_bbox,
            frame_width,
            frame_height,
            alpha=0.22,
            max_center_jump_norm=0.08,
            max_size_ratio=2.0,
        )
        return self._smoothed_bbox


class DetectorBackedTracker:
    """RDV-style detector initialization with OpenCV tracker updates."""

    def __init__(self, tracker_name: str, min_area: float) -> None:
        self._tracker_name = tracker_name
        self._min_area = min_area
        self._tracker: object | None = None
        self._smoothed_bbox: tuple[int, int, int, int] | None = None
        self._misses = 0

    def bbox(self, frame: object) -> tuple[int, int, int, int] | None:
        frame_height, frame_width = frame.shape[:2]
        if self._tracker is None:
            return self._detect_and_initialize(frame)

        detector_bbox = detect_red_target(frame, self._min_area)
        detected, raw_bbox = self._tracker.update(frame)
        if detector_bbox is not None and self._is_detection_consistent(
            detector_bbox,
            frame_width,
            frame_height,
        ):
            self._misses = 0
            return self._initialize_from_detection(detector_bbox, frame, frame_width, frame_height)

        if detected and self._is_tracking_plausible(
            tuple(round(value) for value in raw_bbox),
            frame_width,
            frame_height,
        ):
            self._misses = 0
            self._smoothed_bbox = stabilize_bbox(
                self._smoothed_bbox,
                tuple(round(value) for value in raw_bbox),
                frame_width,
                frame_height,
            )
            return self._smoothed_bbox

        self._misses += 1
        if self._misses >= 2:
            self._tracker = None
        if detector_bbox is not None and self._smoothed_bbox is None:
            return self._initialize_from_detection(detector_bbox, frame, frame_width, frame_height)
        return None

    def _detect_and_initialize(self, frame: object) -> tuple[int, int, int, int] | None:
        detected_bbox = detect_red_target(frame, self._min_area)
        if detected_bbox is None:
            return None

        frame_height, frame_width = frame.shape[:2]
        return self._initialize_from_detection(detected_bbox, frame, frame_width, frame_height)

    def _initialize_from_detection(
        self,
        detected_bbox: tuple[int, int, int, int],
        frame: object,
        frame_width: int,
        frame_height: int,
    ) -> tuple[int, int, int, int] | None:
        initial_bbox = expand_bbox(detected_bbox, 0.18, frame_width, frame_height)
        tracker = create_tracker(self._tracker_name)
        if tracker.init(frame, initial_bbox) is False:
            return None
        self._tracker = tracker
        self._misses = 0
        self._smoothed_bbox = stabilize_bbox(
            self._smoothed_bbox,
            initial_bbox,
            frame_width,
            frame_height,
        )
        return self._smoothed_bbox

    def _is_detection_consistent(
        self,
        detected_bbox: tuple[int, int, int, int],
        frame_width: int,
        frame_height: int,
    ) -> bool:
        if self._smoothed_bbox is None:
            return True
        return bbox_transition_plausible(
            self._smoothed_bbox,
            expand_bbox(detected_bbox, 0.18, frame_width, frame_height),
            frame_width,
            frame_height,
            max_center_jump_norm=0.14,
            max_size_ratio=5.0,
        )

    def _is_tracking_plausible(
        self,
        bbox: tuple[int, int, int, int],
        frame_width: int,
        frame_height: int,
    ) -> bool:
        x, y, width, height = bbox
        if width <= 2 or height <= 2:
            return False
        if x <= 1 or y <= 1 or x + width >= frame_width - 1 or y + height >= frame_height - 1:
            return False
        if self._smoothed_bbox is None:
            return True
        return bbox_transition_plausible(
            self._smoothed_bbox,
            bbox,
            frame_width,
            frame_height,
            max_center_jump_norm=0.16,
            max_size_ratio=6.0,
        )


def bbox_transition_plausible(
    previous: tuple[int, int, int, int],
    current: tuple[int, int, int, int],
    frame_width: int,
    frame_height: int,
    *,
    max_center_jump_norm: float,
    max_size_ratio: float,
) -> bool:
    px, py, pw, ph = previous
    cx, cy, cw, ch = current
    previous_center_x = px + pw / 2.0
    previous_center_y = py + ph / 2.0
    current_center_x = cx + cw / 2.0
    current_center_y = cy + ch / 2.0
    frame_diag = math.hypot(frame_width, frame_height)
    center_jump = math.hypot(
        current_center_x - previous_center_x,
        current_center_y - previous_center_y,
    )
    previous_area = max(1.0, float(pw * ph))
    current_area = max(1.0, float(cw * ch))
    size_ratio = max(previous_area / current_area, current_area / previous_area)
    return (
        center_jump / max(1.0, frame_diag) <= max_center_jump_norm
        and size_ratio <= max_size_ratio
    )


def stabilize_bbox(
    previous: tuple[int, int, int, int] | None,
    current: tuple[int, int, int, int],
    frame_width: int,
    frame_height: int,
    *,
    alpha: float = 0.35,
    max_center_jump_norm: float = 0.22,
    max_size_ratio: float = 2.8,
) -> tuple[int, int, int, int]:
    if previous is None:
        return current

    px, py, pw, ph = previous
    cx, cy, cw, ch = current
    previous_center_x = px + pw / 2.0
    previous_center_y = py + ph / 2.0
    current_center_x = cx + cw / 2.0
    current_center_y = cy + ch / 2.0
    frame_diag = math.hypot(frame_width, frame_height)
    center_jump = math.hypot(
        current_center_x - previous_center_x,
        current_center_y - previous_center_y,
    )
    previous_area = max(1.0, float(pw * ph))
    current_area = max(1.0, float(cw * ch))
    size_ratio = max(previous_area / current_area, current_area / previous_area)
    if center_jump / frame_diag > max_center_jump_norm or size_ratio > max_size_ratio:
        return previous

    smoothed_center_x = previous_center_x + (current_center_x - previous_center_x) * alpha
    smoothed_center_y = previous_center_y + (current_center_y - previous_center_y) * alpha
    smoothed_width = max(1, round(pw + (cw - pw) * alpha))
    smoothed_height = max(1, round(ph + (ch - ph) * alpha))
    return (
        round(clamp(smoothed_center_x - smoothed_width / 2.0, 0, frame_width - smoothed_width)),
        round(clamp(smoothed_center_y - smoothed_height / 2.0, 0, frame_height - smoothed_height)),
        smoothed_width,
        smoothed_height,
    )


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
    if vehicle == "plane":
        if mode != "FBWA":
            fbwa_mode = connection.mode_mapping().get("FBWA")
            if fbwa_mode is None:
                raise RuntimeError("fbwa_mode_unavailable")
            print(
                f"sitl_tracking_status=setting_mode vehicle=plane from={mode} to=FBWA",
                flush=True,
            )
            connection.set_mode(fbwa_mode)
            if not wait_mode(connection, "FBWA", timeout_s):
                raise RuntimeError("fbwa_mode_timeout")
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


def rc_override(connection: mavutil.mavfile, channels: dict[int, int]) -> None:
    values = [0] * 8
    for channel, pwm in channels.items():
        if channel < 1 or channel > 8:
            raise ValueError(f"RC override channel out of range: {channel}")
        values[channel - 1] = round(clamp(pwm, 1000, 2000))
    connection.mav.rc_channels_override_send(
        connection.target_system,
        connection.target_component,
        *values,
    )


def attitude_to_plane_rc_pwm(
    roll_deg: float,
    pitch_deg: float,
    max_roll_deg: float,
    max_pitch_deg: float,
    throttle: float,
    *,
    pitch_rc_reversed: bool = False,
) -> tuple[int, int, int, int]:
    roll_fraction = clamp(roll_deg / max(0.1, max_roll_deg), -1.0, 1.0)
    pitch_fraction = clamp(pitch_deg / max(0.1, max_pitch_deg), -1.0, 1.0)
    if pitch_rc_reversed:
        pitch_fraction = -pitch_fraction
    throttle_fraction = clamp(throttle, 0.0, 1.0)
    return (
        round(1500 + roll_fraction * 500),
        round(1500 + pitch_fraction * 500),
        round(1000 + throttle_fraction * 1000),
        1500,
    )


def plane_throttle_for_pitch(
    pitch_deg: float,
    max_pitch_deg: float,
    neutral_throttle: float,
    min_throttle: float,
    max_throttle: float,
) -> float:
    lower = clamp(min_throttle, 0.0, 1.0)
    upper = clamp(max_throttle, lower, 1.0)
    neutral = clamp(neutral_throttle, lower, upper)
    pitch_fraction = clamp(pitch_deg / max(0.1, max_pitch_deg), -1.0, 1.0)
    if pitch_fraction < 0.0:
        return neutral + pitch_fraction * (neutral - lower)
    return neutral + pitch_fraction * (upper - neutral)


def plane_throttle_for_airspeed(
    airspeed_mps: float | None,
    target_airspeed_mps: float,
    cruise_throttle: float,
    min_throttle: float,
    max_throttle: float,
    airspeed_gain: float,
    pitch_deg: float,
    max_pitch_deg: float,
    target_proximity: float = 0.0,
    near_target_throttle_reduction: float = 0.0,
) -> float:
    lower = clamp(min_throttle, 0.0, 1.0)
    upper = clamp(max_throttle, lower, 1.0)
    throttle = clamp(cruise_throttle, lower, upper)
    proximity = clamp(target_proximity, 0.0, 1.0)
    airspeed_is_plausible = airspeed_mps is not None and (
        airspeed_mps >= 5.0 or proximity < 0.5
    )
    if airspeed_is_plausible:
        throttle += (max(1.0, target_airspeed_mps) - max(0.0, airspeed_mps)) * max(
            0.0,
            airspeed_gain,
        )
    descent_fraction = clamp(-pitch_deg / max(0.1, max_pitch_deg), 0.0, 1.0)
    throttle -= descent_fraction * 0.12
    throttle -= proximity * max(0.0, near_target_throttle_reduction)
    return clamp(throttle, lower, upper)


def plane_pitch_command(
    guided_error_y: float,
    vertical_gain: float,
    far_pitch_gain_scale: float,
    near_pitch_gain_scale: float,
    target_proximity: float,
    below_center_boost: float,
    max_pitch_deg: float,
    near_pitch_down_limit_deg: float | None = None,
) -> float:
    pitch_gain_scale = adaptive_pitch_gain(
        far_pitch_gain_scale,
        near_pitch_gain_scale,
        target_proximity,
    )
    below_center_multiplier = 1.0 + max(0.0, guided_error_y) * max(0.0, below_center_boost)
    pitch_deg = -guided_error_y * vertical_gain * pitch_gain_scale * below_center_multiplier
    if guided_error_y > 0.0 and near_pitch_down_limit_deg is not None:
        near_limit = clamp(near_pitch_down_limit_deg, 0.0, max_pitch_deg)
        allowed_down_pitch = max_pitch_deg - (max_pitch_deg - near_limit) * clamp(
            target_proximity,
            0.0,
            1.0,
        )
        pitch_deg = max(pitch_deg, -allowed_down_pitch)
    return clamp(pitch_deg, -max_pitch_deg, max_pitch_deg)


def clamp_plane_pitch(
    pitch_deg: float,
    max_pitch_up_deg: float,
    max_pitch_down_deg: float,
) -> float:
    return clamp(pitch_deg, -max(0.0, max_pitch_down_deg), max(0.0, max_pitch_up_deg))


def apply_deadband(value: float, deadband: float) -> float:
    return 0.0 if abs(value) <= max(0.0, deadband) else value


def near_target_control_scale(target_proximity: float, minimum_scale: float) -> float:
    proximity = clamp(target_proximity, 0.0, 1.0)
    return 1.0 - proximity * (1.0 - clamp(minimum_scale, 0.0, 1.0))


def far_target_control_scale(target_proximity: float, minimum_scale: float) -> float:
    proximity = clamp(target_proximity, 0.0, 1.0)
    return clamp(minimum_scale, 0.0, 1.0) + proximity * (1.0 - clamp(minimum_scale, 0.0, 1.0))


def scheduled_gain(far_gain: float, near_gain: float, target_proximity: float) -> float:
    proximity = clamp(target_proximity, 0.0, 1.0)
    return max(0.0, far_gain) + (max(0.0, near_gain) - max(0.0, far_gain)) * proximity


def damped_axis_error(
    error: float,
    previous_error: float | None,
    dt_s: float,
    damping_gain: float,
) -> float:
    if previous_error is None or dt_s <= 0.0:
        return error
    error_rate = (error - previous_error) / dt_s
    return error - error_rate * max(0.0, damping_gain)


def adaptive_pitch_gain(
    far_pitch_gain_scale: float,
    near_pitch_gain_scale: float,
    target_proximity: float,
) -> float:
    far = max(0.0, far_pitch_gain_scale)
    near = max(0.0, near_pitch_gain_scale)
    proximity = clamp(target_proximity, 0.0, 1.0)
    return far + (near - far) * proximity


def target_proximity_from_bbox(
    bbox: tuple[int, int, int, int],
    frame_width: int,
    frame_height: int,
    *,
    far_size: float = 0.025,
    near_size: float = 0.16,
) -> float:
    _x, _y, width, height = bbox
    apparent_size = math.sqrt(max(0.0, width * height) / max(1.0, frame_width * frame_height))
    if near_size <= far_size:
        return 0.0
    return clamp((apparent_size - far_size) / (near_size - far_size), 0.0, 1.0)


def damp_pitch_command(
    previous_pitch_deg: float | None,
    target_pitch_deg: float,
    alpha: float,
    max_step_deg: float,
) -> float:
    if previous_pitch_deg is None:
        return target_pitch_deg
    filtered = previous_pitch_deg + (target_pitch_deg - previous_pitch_deg) * clamp(alpha, 0.0, 1.0)
    max_step = max(0.0, max_step_deg)
    return clamp(filtered, previous_pitch_deg - max_step, previous_pitch_deg + max_step)


def send_plane_rc_attitude(
    connection: mavutil.mavfile,
    roll_deg: float,
    pitch_deg: float,
    max_roll_deg: float,
    max_pitch_deg: float,
    throttle: float,
    pitch_rc_reversed: bool = False,
) -> None:
    roll_pwm, pitch_pwm, throttle_pwm, yaw_pwm = attitude_to_plane_rc_pwm(
        roll_deg,
        pitch_deg,
        max_roll_deg,
        max_pitch_deg,
        throttle,
        pitch_rc_reversed=pitch_rc_reversed,
    )
    rc_override(
        connection,
        {
            1: roll_pwm,
            2: pitch_pwm,
            3: throttle_pwm,
            4: yaw_pwm,
        },
    )


def release_rc_override(connection: mavutil.mavfile) -> None:
    connection.mav.rc_channels_override_send(
        connection.target_system,
        connection.target_component,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
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


def mavlink_param_name(message: object) -> str:
    raw_name = getattr(message, "param_id", "")
    if isinstance(raw_name, bytes):
        return raw_name.decode("ascii", errors="ignore").rstrip("\x00")
    return str(raw_name).rstrip("\x00")


def read_plane_parameters(
    connection: mavutil.mavfile,
    target_system: int,
    target_component: int,
    names: tuple[str, ...] = PLANE_RESPONSE_PARAM_NAMES,
    *,
    timeout_per_param_s: float = 0.12,
) -> dict[str, float]:
    values: dict[str, float] = {}
    timeout_s = max(0.02, timeout_per_param_s)
    for name in names:
        connection.mav.param_request_read_send(
            target_system,
            target_component,
            name.encode("ascii"),
            -1,
        )
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            message = connection.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.02)
            if message is None:
                continue
            param_name = mavlink_param_name(message)
            if param_name in names:
                values[param_name] = float(message.param_value)
            if param_name == name:
                break
    return values


def plane_response_model_from_params(
    args: argparse.Namespace,
    values: dict[str, float],
) -> PlaneResponseModel:
    max_roll_deg = args.max_plane_roll_deg
    if "LIM_ROLL_CD" in values and values["LIM_ROLL_CD"] > 0:
        max_roll_deg = min(max_roll_deg, values["LIM_ROLL_CD"] / 100.0)

    max_pitch_up_deg = args.max_plane_pitch_deg
    if "LIM_PITCH_MAX" in values and values["LIM_PITCH_MAX"] > 0:
        max_pitch_up_deg = min(max_pitch_up_deg, values["LIM_PITCH_MAX"] / 100.0)

    max_pitch_down_deg = args.max_plane_pitch_deg
    if "LIM_PITCH_MIN" in values:
        max_pitch_down_deg = min(max_pitch_down_deg, abs(values["LIM_PITCH_MIN"]) / 100.0)

    cruise_throttle = args.plane_throttle
    if "TRIM_THROTTLE" in values:
        cruise_throttle = clamp(values["TRIM_THROTTLE"] / 100.0, 0.0, 1.0)

    min_throttle = args.plane_min_throttle
    if "THR_MIN" in values:
        min_throttle = clamp(values["THR_MIN"] / 100.0, 0.0, 1.0)

    max_throttle = args.plane_max_throttle
    if "THR_MAX" in values:
        max_throttle = clamp(values["THR_MAX"] / 100.0, min_throttle, 1.0)

    target_airspeed_mps = args.plane_airspeed_mps
    if "ARSPD_FBW_MIN" in values:
        target_airspeed_mps = max(target_airspeed_mps, values["ARSPD_FBW_MIN"])
    if "ARSPD_FBW_MAX" in values and values["ARSPD_FBW_MAX"] > 0:
        target_airspeed_mps = min(target_airspeed_mps, values["ARSPD_FBW_MAX"])

    pitch_filter_alpha = args.plane_pitch_filter_alpha
    if "PTCH2SRV_TCONST" in values and values["PTCH2SRV_TCONST"] > 0:
        period_s = 1.0 / max(1.0, args.rate_hz)
        response_alpha = period_s / (values["PTCH2SRV_TCONST"] + period_s)
        pitch_filter_alpha = clamp(response_alpha, 0.08, args.plane_pitch_filter_alpha)

    max_pitch_step_deg = args.plane_max_pitch_step_deg
    if "PTCH2SRV_TCONST" in values and values["PTCH2SRV_TCONST"] > 0:
        max_pitch_step_deg = clamp(
            args.plane_max_pitch_step_deg / values["PTCH2SRV_TCONST"],
            1.0,
            args.plane_max_pitch_step_deg,
        )

    pitch_channel = round(values.get("RCMAP_PITCH", 2.0))
    pitch_rc_reversed = bool(
        pitch_channel == 2
        and "RC2_REVERSED" in values
        and round(values["RC2_REVERSED"]) != 0
    )

    return PlaneResponseModel(
        max_roll_deg=max(1.0, max_roll_deg),
        max_pitch_up_deg=max(1.0, max_pitch_up_deg),
        max_pitch_down_deg=max(1.0, max_pitch_down_deg),
        cruise_throttle=clamp(cruise_throttle, min_throttle, max_throttle),
        min_throttle=min_throttle,
        max_throttle=max_throttle,
        target_airspeed_mps=max(1.0, target_airspeed_mps),
        pitch_filter_alpha=pitch_filter_alpha,
        max_pitch_step_deg=max_pitch_step_deg,
        pitch_rc_reversed=pitch_rc_reversed,
        raw_params=values,
    )


def read_plane_telemetry(
    connection: mavutil.mavfile,
    last_heading_deg: float,
    last_relative_alt_m: float,
    last_airspeed_mps: float | None,
) -> tuple[float, float, float | None]:
    deadline = time.monotonic() + 0.03
    heading_deg = last_heading_deg
    relative_alt_m = last_relative_alt_m
    airspeed_mps = last_airspeed_mps
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
            airspeed_mps = float(message.airspeed)
        elif message.get_type() == "GLOBAL_POSITION_INT":
            relative_alt_m = float(message.relative_alt) / 1000.0
    return heading_deg, relative_alt_m, airspeed_mps


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

    connection = open_mavlink_connection(
        args.mavlink,
        source_system=args.source_system,
        source_component=args.source_component,
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
    detector_tracker = None
    if args.tracking_mode == "custom":
        if args.selection_file is None:
            print("sitl_tracking_status=failed reason=missing_custom_selection_file", flush=True)
            return 2
        custom_tracker = CustomSelectionTracker(args.tracker, args.selection_file)
    else:
        detector_tracker = DetectorBackedTracker(args.tracker, args.min_area)

    deadline = time.monotonic() + args.timeout_s
    period_s = 1.0 / args.rate_hz
    frames = 0
    detections = 0
    last_report = 0.0
    last_command = time.monotonic()
    last_detection_time: float | None = None
    plane_heading_deg = 0.0
    plane_relative_alt_m = 50.0
    plane_airspeed_mps: float | None = None
    previous_error_x: float | None = None
    previous_error_y: float | None = None
    previous_error_time: float | None = None
    previous_roll_command_deg: float | None = 0.0 if vehicle == "plane" else None
    previous_pitch_command_deg: float | None = 0.0 if vehicle == "plane" else None
    throttle_report = 0.0
    pitch_gain_report = args.plane_pitch_gain_scale
    proximity_report = 0.0
    control_scale_report = 1.0
    centering_gain_report = args.plane_centering_gain
    damping_gain_report = args.plane_damping_gain
    airspeed_report: float | None = None
    pitch_pwm_report = 1500
    plane_response_model = plane_response_model_from_params(args, {})
    if vehicle == "plane":
        if args.read_plane_params:
            param_values = read_plane_parameters(
                connection,
                target_system,
                target_component,
                timeout_per_param_s=args.plane_param_timeout_s,
            )
            plane_response_model = plane_response_model_from_params(args, param_values)
            print(
                "sitl_tracking_status=plane_params_read "
                f"count={len(param_values)} "
                f"max_roll_deg={plane_response_model.max_roll_deg:.2f} "
                f"max_pitch_up_deg={plane_response_model.max_pitch_up_deg:.2f} "
                f"max_pitch_down_deg={plane_response_model.max_pitch_down_deg:.2f} "
                f"target_airspeed_mps={plane_response_model.target_airspeed_mps:.2f} "
                f"pitch_filter_alpha={plane_response_model.pitch_filter_alpha:.2f} "
                f"max_pitch_step_deg={plane_response_model.max_pitch_step_deg:.2f} "
                f"pitch_rc_reversed={int(plane_response_model.pitch_rc_reversed)}",
                flush=True,
            )
        plane_heading_deg, plane_relative_alt_m, plane_airspeed_mps = read_plane_telemetry(
            connection,
            plane_heading_deg,
            plane_relative_alt_m,
            plane_airspeed_mps,
        )
        send_plane_speed(
            connection,
            target_system,
            target_component,
            plane_response_model.target_airspeed_mps,
        )
        throttle_report = plane_throttle_for_airspeed(
            plane_airspeed_mps,
            plane_response_model.target_airspeed_mps,
            plane_response_model.cruise_throttle,
            plane_response_model.min_throttle,
            plane_response_model.max_throttle,
            args.plane_throttle_airspeed_gain,
            0.0,
            max(plane_response_model.max_pitch_up_deg, plane_response_model.max_pitch_down_deg),
        )
        airspeed_report = plane_airspeed_mps
        send_plane_rc_attitude(
            connection,
            0.0,
            0.0,
            plane_response_model.max_roll_deg,
            max(plane_response_model.max_pitch_up_deg, plane_response_model.max_pitch_down_deg),
            throttle_report,
            plane_response_model.pitch_rc_reversed,
        )

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
                    else detector_tracker.bbox(frame)
                )
            except RuntimeError as exc:
                print(f"sitl_tracking_status=failed reason={exc}", flush=True)
                return 1
            now = time.monotonic()
            if bbox is None:
                hold_attitude = False
                if vehicle == "quad":
                    send_body_velocity(connection, target_system, target_component, 0, 0, 0, 0)
                else:
                    previous_error_x = None
                    previous_error_y = None
                    previous_error_time = None
                    plane_heading_deg, plane_relative_alt_m, plane_airspeed_mps = (
                        read_plane_telemetry(
                            connection,
                            plane_heading_deg,
                            plane_relative_alt_m,
                            plane_airspeed_mps,
                        )
                    )
                    airspeed_report = plane_airspeed_mps
                    send_plane_speed(
                        connection,
                        target_system,
                        target_component,
                        plane_response_model.target_airspeed_mps,
                    )
                    hold_age_s = (
                        float("inf") if last_detection_time is None else now - last_detection_time
                    )
                    hold_attitude = (
                        previous_roll_command_deg is not None
                        and previous_pitch_command_deg is not None
                        and hold_age_s <= max(0.0, args.plane_loss_hold_s)
                    )
                    if not hold_attitude:
                        previous_roll_command_deg = 0.0
                        previous_pitch_command_deg = 0.0
                        throttle_report = plane_throttle_for_airspeed(
                            plane_airspeed_mps,
                            plane_response_model.target_airspeed_mps,
                            plane_response_model.cruise_throttle,
                            plane_response_model.min_throttle,
                            plane_response_model.max_throttle,
                            args.plane_throttle_airspeed_gain,
                            0.0,
                            max(
                                plane_response_model.max_pitch_up_deg,
                                plane_response_model.max_pitch_down_deg,
                            ),
                        )
                    send_plane_rc_attitude(
                        connection,
                        previous_roll_command_deg,
                        previous_pitch_command_deg,
                        plane_response_model.max_roll_deg,
                        max(
                            plane_response_model.max_pitch_up_deg,
                            plane_response_model.max_pitch_down_deg,
                        ),
                        throttle_report,
                        plane_response_model.pitch_rc_reversed,
                    )
                    _roll_pwm, pitch_pwm_report, _throttle_pwm, _yaw_pwm = attitude_to_plane_rc_pwm(
                        previous_roll_command_deg,
                        previous_pitch_command_deg,
                        plane_response_model.max_roll_deg,
                        max(
                            plane_response_model.max_pitch_up_deg,
                            plane_response_model.max_pitch_down_deg,
                        ),
                        throttle_report,
                        pitch_rc_reversed=plane_response_model.pitch_rc_reversed,
                    )
                if now - last_report >= 1.0:
                    loss_behavior = (
                        "holding_last_attitude"
                        if vehicle == "plane" and hold_attitude
                        else "neutral_search"
                    )
                    print(
                        f"sitl_tracking_status=searching mode={args.tracking_mode} "
                        "reason=target_not_detected "
                        f"loss_behavior={loss_behavior} "
                        f"loss_hold_s={args.plane_loss_hold_s:.2f}",
                        flush=True,
                    )
                    last_report = now
                time.sleep(period_s)
                continue

            detections += 1
            last_detection_time = now
            x, y, width, height = bbox
            frame_height, frame_width = frame.shape[:2]
            target_proximity = target_proximity_from_bbox(bbox, frame_width, frame_height)
            center_x = (x + width / 2) / frame_width
            center_y = (y + height / 2) / frame_height
            error_x = center_x - 0.5
            error_y = center_y - 0.5
            if vehicle == "plane":
                aspect_ratio = frame_width / frame_height
                vertical_fov_deg = math.degrees(
                    2.0
                    * math.atan(
                        math.tan(math.radians(args.plane_camera_hfov_deg) * 0.5)
                        / aspect_ratio
                    )
                )
                guided_error_x = perspective_correct_error(
                    error_x,
                    args.plane_camera_hfov_deg,
                )
                guided_error_y = perspective_correct_error(error_y, vertical_fov_deg)
            else:
                guided_error_x = error_x
                guided_error_y = error_y
            if vehicle == "plane":
                guided_error_x = apply_deadband(guided_error_x, args.plane_error_deadband)
                guided_error_y = apply_deadband(guided_error_y, args.plane_error_deadband)
            proportional_error_x = guided_error_x
            proportional_error_y = guided_error_y
            dt = 0.0
            if (
                vehicle == "plane"
                and previous_error_x is not None
                and previous_error_y is not None
                and previous_error_time is not None
            ):
                dt = max(0.001, now - previous_error_time)
                damping_gain = scheduled_gain(
                    args.plane_damping_gain,
                    args.plane_near_damping_gain,
                    target_proximity,
                )
                guided_error_x = damped_axis_error(
                    proportional_error_x,
                    previous_error_x,
                    dt,
                    damping_gain,
                )
                guided_error_y = damped_axis_error(
                    proportional_error_y,
                    previous_error_y,
                    dt,
                    damping_gain,
                )
                damping_gain_report = damping_gain
            previous_error_x = proportional_error_x
            previous_error_y = proportional_error_y
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
                plane_heading_deg, plane_relative_alt_m, plane_airspeed_mps = read_plane_telemetry(
                    connection,
                    plane_heading_deg,
                    plane_relative_alt_m,
                    plane_airspeed_mps,
                )
                far_scale = far_target_control_scale(
                    target_proximity,
                    args.plane_far_control_scale,
                )
                near_scale = near_target_control_scale(
                    target_proximity,
                    args.plane_near_control_scale,
                )
                control_scale = far_scale * near_scale
                control_scale_report = control_scale
                centering_gain = scheduled_gain(
                    args.plane_centering_gain,
                    args.plane_near_centering_gain,
                    target_proximity,
                )
                centering_gain_report = centering_gain
                roll_deg = clamp(
                    guided_error_x
                    * args.vertical_gain
                    * centering_gain
                    * args.plane_roll_gain_scale
                    * control_scale,
                    -plane_response_model.max_roll_deg,
                    plane_response_model.max_roll_deg,
                )
                roll_deg = damp_pitch_command(
                    previous_roll_command_deg,
                    roll_deg,
                    plane_response_model.pitch_filter_alpha,
                    args.plane_max_roll_step_deg,
                )
                previous_roll_command_deg = roll_deg
                pitch_deg = plane_pitch_command(
                    guided_error_y,
                    args.vertical_gain * centering_gain * control_scale,
                    args.plane_pitch_gain_scale,
                    args.plane_pitch_near_gain_scale,
                    target_proximity,
                    args.plane_pitch_below_center_boost,
                    max(
                        plane_response_model.max_pitch_up_deg,
                        plane_response_model.max_pitch_down_deg,
                    ),
                    args.plane_near_pitch_down_limit_deg,
                )
                pitch_deg = clamp_plane_pitch(
                    pitch_deg,
                    plane_response_model.max_pitch_up_deg,
                    plane_response_model.max_pitch_down_deg,
                )
                pitch_deg = damp_pitch_command(
                    previous_pitch_command_deg,
                    pitch_deg,
                    plane_response_model.pitch_filter_alpha,
                    plane_response_model.max_pitch_step_deg,
                )
                previous_pitch_command_deg = pitch_deg
                pitch_gain_report = adaptive_pitch_gain(
                    args.plane_pitch_gain_scale,
                    args.plane_pitch_near_gain_scale,
                    target_proximity,
                )
                proximity_report = target_proximity
                throttle_report = plane_throttle_for_airspeed(
                    plane_airspeed_mps,
                    plane_response_model.target_airspeed_mps,
                    plane_response_model.cruise_throttle,
                    plane_response_model.min_throttle,
                    plane_response_model.max_throttle,
                    args.plane_throttle_airspeed_gain,
                    pitch_deg,
                    max(
                        plane_response_model.max_pitch_up_deg,
                        plane_response_model.max_pitch_down_deg,
                    ),
                    target_proximity,
                    args.plane_near_throttle_reduction,
                )
                airspeed_report = plane_airspeed_mps
                send_plane_speed(
                    connection,
                    target_system,
                    target_component,
                    plane_response_model.target_airspeed_mps,
                )
                pitch_report = pitch_deg
                roll_report = roll_deg
                send_plane_rc_attitude(
                    connection,
                    roll_deg,
                    pitch_deg,
                    plane_response_model.max_roll_deg,
                    max(
                        plane_response_model.max_pitch_up_deg,
                        plane_response_model.max_pitch_down_deg,
                    ),
                    throttle_report,
                    plane_response_model.pitch_rc_reversed,
                )
                _roll_pwm, pitch_pwm_report, _throttle_pwm, _yaw_pwm = attitude_to_plane_rc_pwm(
                    roll_deg,
                    pitch_deg,
                    plane_response_model.max_roll_deg,
                    max(
                        plane_response_model.max_pitch_up_deg,
                        plane_response_model.max_pitch_down_deg,
                    ),
                    throttle_report,
                    pitch_rc_reversed=plane_response_model.pitch_rc_reversed,
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
                    f"pitch_pwm={pitch_pwm_report} "
                    f"pitch_rc_reversed={int(plane_response_model.pitch_rc_reversed)} "
                    f"guided_error={guided_error_x:.3f},{guided_error_y:.3f} "
                    f"target_proximity={proximity_report:.2f} "
                    f"near_control_scale={control_scale_report:.2f} "
                    f"centering_gain={centering_gain_report:.2f} "
                    f"damping_gain={damping_gain_report:.2f} "
                    f"pitch_gain={pitch_gain_report:.2f} "
                    f"pitch_below_boost={args.plane_pitch_below_center_boost:.2f} "
                    f"near_pitch_down_limit_deg={args.plane_near_pitch_down_limit_deg:.2f} "
                    f"near_throttle_reduction={args.plane_near_throttle_reduction:.2f} "
                    f"pitch_filter_alpha={plane_response_model.pitch_filter_alpha:.2f} "
                    f"pitch_step_deg={plane_response_model.max_pitch_step_deg:.2f} "
                    f"airspeed_mps={airspeed_report if airspeed_report is not None else -1.0:.2f} "
                    f"target_airspeed_mps={plane_response_model.target_airspeed_mps:.2f} "
                    f"throttle={throttle_report:.2f} "
                    f"control=fbwa_rc",
                    flush=True,
                )
                last_report = now

            sleep_s = max(0.0, period_s - (time.monotonic() - last_command))
            time.sleep(sleep_s)
    finally:
        if vehicle == "quad":
            send_body_velocity(connection, target_system, target_component, 0, 0, 0, 0)
        else:
            release_rc_override(connection)
        if capture is not None:
            capture.release()

    print(
        f"sitl_tracking_status=stopped camera_frames={frames} target_detections={detections}",
        flush=True,
    )
    return 0 if detections > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
