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
from collections.abc import Callable
from pathlib import Path
from typing import NamedTuple

import cv2
from mavlink_endpoint import open_mavlink_connection
from pymavlink import mavutil
from track_camera_target import (
    detect_banner_target,
    detect_colored_target,
    newest_image,
    open_capture,
)

from vulture_x.vision.tracker import (
    TemplateMatchingTracker,
    expand_bbox,
    refine_bbox_to_salient_region,
)

DEFAULT_MAVLINK = "udpin:0.0.0.0:14550"
STABLE_FRAME_MIN_AGE_S = 0.02
PLANE_TARGET_AIRSPEED_MPS = 20.0
PLANE_CRUISE_THROTTLE = 0.55
DEFAULT_MAX_FRAME_AGE_MS = 750.0
DEFAULT_MAVLINK_HEARTBEAT_TIMEOUT_S = 0.0
DEFAULT_MIN_TRACKING_ALT_M = 15.0
DEFAULT_PLANE_PROXIMITY_FAR_SIZE = 0.025
DEFAULT_PLANE_PROXIMITY_NEAR_SIZE = 0.16
MAX_GUIDED_IMAGE_ERROR = 1.0
MIN_PLANE_COMMAND_FILTER_ALPHA = 0.18
PLANE_RESPONSE_PARAM_NAMES = (
    "ROLL_LIMIT_DEG",
    "PTCH_LIM_MAX_DEG",
    "PTCH_LIM_MIN_DEG",
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
    "RCMAP_ROLL",
    "RCMAP_PITCH",
    "RCMAP_THROTTLE",
    "RCMAP_YAW",
    "MAV_GCS_SYSID",
    "MAV_GCS_SYSID_HI",
    "MAV_OPTIONS",
    "RC_OPTIONS",
    "RC_OVERRIDE_TIME",
    *(f"RC{channel}_{suffix}" for channel in range(1, 9) for suffix in ("MIN", "TRIM", "MAX")),
    "RC1_REVERSED",
    "RC2_REVERSED",
    "RC3_REVERSED",
    "RC4_REVERSED",
    "SERVO1_FUNCTION",
    "SERVO2_FUNCTION",
    "SERVO3_FUNCTION",
    "SERVO4_FUNCTION",
    "SERVO5_FUNCTION",
    "SERVO6_FUNCTION",
    "SERVO7_FUNCTION",
    "SERVO8_FUNCTION",
)


class PlaneTrackingTuning(NamedTuple):
    revision: int
    vertical_gain: float
    plane_centering_gain: float
    plane_near_centering_gain: float
    plane_roll_gain_scale: float
    plane_pitch_gain_scale: float
    plane_pitch_near_gain_scale: float
    plane_error_deadband: float
    plane_lead_s: float
    plane_damping_gain: float
    plane_near_damping_gain: float
    plane_pitch_filter_alpha: float
    plane_max_pitch_step_deg: float
    plane_max_roll_step_deg: float
    max_plane_roll_deg: float
    max_plane_pitch_deg: float
    plane_near_pitch_down_limit_deg: float
    plane_far_control_scale: float
    plane_near_control_scale: float
    plane_camera_hfov_deg: float
    plane_proximity_far_size: float
    plane_proximity_near_size: float
    plane_airspeed_mps: float
    plane_throttle: float
    plane_throttle_airspeed_gain: float
    plane_min_throttle: float
    plane_max_throttle: float
    plane_near_throttle_reduction: float
    plane_pitch_below_center_boost: float
    plane_loss_hold_s: float
    min_tracking_alt_m: float
    airspeed_low_persistence_s: float


TUNING_RANGES: dict[str, tuple[float, float]] = {
    "vertical_gain": (0.0, 80.0),
    "plane_centering_gain": (0.0, 4.0),
    "plane_near_centering_gain": (0.0, 4.0),
    "plane_roll_gain_scale": (0.0, 3.0),
    "plane_pitch_gain_scale": (0.0, 3.0),
    "plane_pitch_near_gain_scale": (0.0, 5.0),
    "plane_error_deadband": (0.0, 0.2),
    "plane_lead_s": (0.0, 1.0),
    "plane_damping_gain": (0.0, 3.0),
    "plane_near_damping_gain": (0.0, 3.0),
    "plane_pitch_filter_alpha": (0.05, 0.8),
    "plane_max_pitch_step_deg": (1.0, 8.0),
    "plane_max_roll_step_deg": (1.0, 10.0),
    "max_plane_roll_deg": (1.0, 45.0),
    "max_plane_pitch_deg": (1.0, 45.0),
    "plane_near_pitch_down_limit_deg": (0.0, 45.0),
    "plane_far_control_scale": (0.1, 1.0),
    "plane_near_control_scale": (0.1, 1.0),
    "plane_camera_hfov_deg": (20.0, 140.0),
    "plane_proximity_far_size": (0.001, 0.5),
    "plane_proximity_near_size": (0.002, 0.8),
    "plane_airspeed_mps": (5.0, 40.0),
    "plane_throttle": (0.0, 1.0),
    "plane_throttle_airspeed_gain": (0.0, 0.2),
    "plane_min_throttle": (0.0, 1.0),
    "plane_max_throttle": (0.0, 1.0),
    "plane_near_throttle_reduction": (0.0, 0.5),
    "plane_pitch_below_center_boost": (0.0, 3.0),
    "plane_loss_hold_s": (0.0, 5.0),
    "min_tracking_alt_m": (0.0, 200.0),
    "airspeed_low_persistence_s": (0.2, 10.0),
}


class TuningFileState(NamedTuple):
    path: Path | None
    mtime_ns: int | None = None


class RcCalibration(NamedTuple):
    minimum: int = 1000
    trim: int = 1500
    maximum: int = 2000
    reversed: bool = False


class RcCalibrationSet(NamedTuple):
    roll: RcCalibration = RcCalibration()
    pitch: RcCalibration = RcCalibration()
    throttle: RcCalibration = RcCalibration()
    yaw: RcCalibration = RcCalibration()


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
    roll_rc_reversed: bool
    pitch_rc_reversed: bool
    roll_channel: int
    pitch_channel: int
    throttle_channel: int
    yaw_channel: int
    rc_calibration: RcCalibrationSet
    min_airspeed_mps: float | None
    rc_override_timeout_s: float | None
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
    parser.add_argument(
        "--timeout-s",
        type=float,
        default=0.0,
        help=(
            "Deprecated and ignored. Tracking must stop only on manual stop or "
            "target-loss safety logic."
        ),
    )
    parser.add_argument("--min-area", type=float, default=25.0)
    parser.add_argument("--rate-hz", type=float, default=30.0)
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
        default=1.55,
        help="Extra fixed-wing roll/pitch gain for keeping the target on the crosshair.",
    )
    parser.add_argument(
        "--plane-near-centering-gain",
        type=float,
        default=2.65,
        help="Extra fixed-wing centering gain as bbox proximity approaches near-target.",
    )
    parser.add_argument(
        "--plane-damping-gain",
        type=float,
        default=0.14,
        help="Fixed-wing damping gain against image-error rate while target is far.",
    )
    parser.add_argument(
        "--plane-near-damping-gain",
        type=float,
        default=0.30,
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
        default=1.75,
        help="Internal fixed-wing roll gain multiplier for camera frame centering.",
    )
    parser.add_argument(
        "--plane-pitch-gain-scale",
        type=float,
        default=1.20,
        help="Fixed-wing pitch gain multiplier when the target appears far away.",
    )
    parser.add_argument(
        "--plane-pitch-near-gain-scale",
        type=float,
        default=1.60,
        help="Fixed-wing pitch gain multiplier when bbox size indicates a near target.",
    )
    parser.add_argument(
        "--plane-far-control-scale",
        type=float,
        default=0.72,
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
        default=0.45,
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
        default=6.0,
        help="Maximum fixed-wing roll command change per camera update.",
    )
    parser.add_argument(
        "--plane-loss-hold-s",
        type=float,
        default=1.5,
        help="Bounded time to keep the last fixed-wing attitude command after target loss.",
    )
    parser.add_argument(
        "--surface-test-min-deflection-deg",
        type=float,
        default=8.0,
        help=(
            "Plane surface-test only: minimum visible roll/pitch demand when the "
            "tracked target is outside the image deadband."
        ),
    )
    parser.add_argument(
        "--surface-test-throttle",
        type=float,
        default=0.0,
        help=(
            "Plane surface-test throttle fraction. Keep the default at zero for "
            "manual hardware tests; the simulator UI may raise this explicitly."
        ),
    )
    parser.add_argument(
        "--demand-state-file",
        type=Path,
        help="Optional JSON file updated every command cycle for UI demand overlays.",
    )
    parser.add_argument(
        "--tuning-file",
        type=Path,
        help="Optional JSON file for live fixed-wing tuning updates.",
    )
    parser.add_argument(
        "--max-frame-age-ms",
        type=float,
        default=DEFAULT_MAX_FRAME_AGE_MS,
        help="Maximum accepted camera-frame age before fixed-wing steering releases RC override.",
    )
    parser.add_argument(
        "--mavlink-heartbeat-timeout-s",
        type=float,
        default=DEFAULT_MAVLINK_HEARTBEAT_TIMEOUT_S,
        help=(
            "Deprecated and ignored. Tracking stops only on manual stop or "
            "configured target-loss logic."
        ),
    )
    parser.add_argument(
        "--simulator-mode",
        action="store_true",
        help="Allow SITL-only startup conveniences such as switching ArduPlane to FBWA.",
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
        "--plane-proximity-far-size",
        type=float,
        default=DEFAULT_PLANE_PROXIMITY_FAR_SIZE,
        help="Apparent bbox-size threshold treated as far target.",
    )
    parser.add_argument(
        "--plane-proximity-near-size",
        type=float,
        default=DEFAULT_PLANE_PROXIMITY_NEAR_SIZE,
        help="Apparent bbox-size threshold treated as near target.",
    )
    parser.add_argument(
        "--min-tracking-alt-m",
        type=float,
        default=DEFAULT_MIN_TRACKING_ALT_M,
        help="Minimum relative altitude gate for fixed-wing active tracking.",
    )
    parser.add_argument(
        "--airspeed-low-persistence-s",
        type=float,
        default=2.0,
        help="Duration below aircraft minimum airspeed before releasing fixed-wing steering.",
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
        "--plane-param-cache-file",
        type=Path,
        help="Optional per-session plane parameter cache to skip repeated param reads.",
    )
    parser.add_argument(
        "--plane-lead-s",
        type=float,
        default=0.0,
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
        choices=("banner", "red", "orange", "custom", "head", "person"),
        default="red",
        help=(
            "red detects a red/orange physical target; banner detects the Gazebo "
            "tracking banner; orange is a legacy alias for red; custom/head track "
            "a UI-selected ROI."
        ),
    )
    parser.add_argument("--selection-file", type=Path, default=None)
    parser.add_argument("--tracker", choices=("CSRT", "KCF", "TEMPLATE"), default="TEMPLATE")
    parser.add_argument(
        "--enable-guidance",
        action="store_true",
        help="Required. Send bounded commands to local ArduPilot SITL.",
    )
    parser.add_argument(
        "--surface-test",
        action="store_true",
        help=(
            "Plane-only test: track the target in FBWA and move roll/pitch "
            "surfaces. Throttle defaults to zero unless --surface-test-throttle is set."
        ),
    )
    args = parser.parse_args()
    args.timeout_s = 0.0
    args.mavlink_heartbeat_timeout_s = 0.0
    return args


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def plane_tuning_from_args(args: argparse.Namespace) -> PlaneTrackingTuning:
    return clamp_plane_tuning(
        PlaneTrackingTuning(
            revision=0,
            vertical_gain=float(args.vertical_gain),
            plane_centering_gain=float(args.plane_centering_gain),
            plane_near_centering_gain=float(args.plane_near_centering_gain),
            plane_roll_gain_scale=float(args.plane_roll_gain_scale),
            plane_pitch_gain_scale=float(args.plane_pitch_gain_scale),
            plane_pitch_near_gain_scale=float(args.plane_pitch_near_gain_scale),
            plane_error_deadband=float(args.plane_error_deadband),
            plane_lead_s=float(args.plane_lead_s),
            plane_damping_gain=float(args.plane_damping_gain),
            plane_near_damping_gain=float(args.plane_near_damping_gain),
            plane_pitch_filter_alpha=float(args.plane_pitch_filter_alpha),
            plane_max_pitch_step_deg=float(args.plane_max_pitch_step_deg),
            plane_max_roll_step_deg=float(args.plane_max_roll_step_deg),
            max_plane_roll_deg=float(args.max_plane_roll_deg),
            max_plane_pitch_deg=float(args.max_plane_pitch_deg),
            plane_near_pitch_down_limit_deg=float(args.plane_near_pitch_down_limit_deg),
            plane_far_control_scale=float(args.plane_far_control_scale),
            plane_near_control_scale=float(args.plane_near_control_scale),
            plane_camera_hfov_deg=float(args.plane_camera_hfov_deg),
            plane_proximity_far_size=float(args.plane_proximity_far_size),
            plane_proximity_near_size=float(args.plane_proximity_near_size),
            plane_airspeed_mps=float(args.plane_airspeed_mps),
            plane_throttle=float(args.plane_throttle),
            plane_throttle_airspeed_gain=float(args.plane_throttle_airspeed_gain),
            plane_min_throttle=float(args.plane_min_throttle),
            plane_max_throttle=float(args.plane_max_throttle),
            plane_near_throttle_reduction=float(args.plane_near_throttle_reduction),
            plane_pitch_below_center_boost=float(args.plane_pitch_below_center_boost),
            plane_loss_hold_s=float(args.plane_loss_hold_s),
            min_tracking_alt_m=float(args.min_tracking_alt_m),
            airspeed_low_persistence_s=float(args.airspeed_low_persistence_s),
        )
    )


def clamp_plane_tuning(tuning: PlaneTrackingTuning) -> PlaneTrackingTuning:
    values = tuning._asdict()
    for key, bounds in TUNING_RANGES.items():
        values[key] = clamp(float(values[key]), bounds[0], bounds[1])
    if values["plane_proximity_near_size"] <= values["plane_proximity_far_size"]:
        values["plane_proximity_near_size"] = min(
            TUNING_RANGES["plane_proximity_near_size"][1],
            values["plane_proximity_far_size"] + 0.001,
        )
    if values["plane_max_throttle"] < values["plane_min_throttle"]:
        values["plane_max_throttle"] = values["plane_min_throttle"]
    values["revision"] = int(values["revision"])
    return PlaneTrackingTuning(**values)


def update_tuning_from_file(
    state: TuningFileState,
    current: PlaneTrackingTuning,
) -> tuple[TuningFileState, PlaneTrackingTuning]:
    if state.path is None:
        return state, current
    try:
        stat = state.path.stat()
    except FileNotFoundError:
        return TuningFileState(state.path, None), current
    except OSError as exc:
        print(
            f"tracking_tuning_status=ignored reason=stat_failed detail={type(exc).__name__}",
            flush=True,
        )
        return state, current
    if stat.st_mtime_ns == state.mtime_ns:
        return state, current
    try:
        payload = json.loads(state.path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        print("tracking_tuning_status=ignored reason=invalid_json", flush=True)
        return TuningFileState(state.path, stat.st_mtime_ns), current
    except OSError as exc:
        print(
            f"tracking_tuning_status=ignored reason=read_failed detail={type(exc).__name__}",
            flush=True,
        )
        return state, current
    if not isinstance(payload, dict):
        print("tracking_tuning_status=ignored reason=invalid_payload", flush=True)
        return TuningFileState(state.path, stat.st_mtime_ns), current
    raw_values = payload.get("values", payload)
    if not isinstance(raw_values, dict):
        print("tracking_tuning_status=ignored reason=invalid_values", flush=True)
        return TuningFileState(state.path, stat.st_mtime_ns), current
    updates: dict[str, float] = {}
    for key in TUNING_RANGES:
        if key not in raw_values:
            continue
        try:
            updates[key] = float(raw_values[key])
        except (TypeError, ValueError):
            print(f"tracking_tuning_status=ignored reason=invalid_value field={key}", flush=True)
            return TuningFileState(state.path, stat.st_mtime_ns), current
    revision = int(payload.get("revision", current.revision + 1))
    next_tuning = clamp_plane_tuning(current._replace(revision=revision, **updates))
    print(f"tracking_tuning_status=applied revision={next_tuning.revision}", flush=True)
    return TuningFileState(state.path, stat.st_mtime_ns), next_tuning


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


def create_manual_tracker(tracker_name: str) -> object:
    if tracker_name == "TEMPLATE":
        return TemplateMatchingTracker(
            search_margin_px=80,
            minimum_match_score=0.38,
            template_update_alpha=0.06,
            bbox_update_alpha=0.50,
            max_center_jump_norm=0.18,
            max_size_ratio=4.0,
            scale_factors=(0.88, 0.95, 1.0, 1.06, 1.14),
            grayscale=True,
            foreground_refinement=False,
        )
    return create_tracker(tracker_name)


def selection_bbox(selection_file: Path, frame: object) -> tuple[int, int, int, int]:
    raw = json.loads(selection_file.read_text(encoding="utf-8"))
    if raw.get("mode") not in {"custom", "head", "person"}:
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
    return refine_bbox_to_salient_region(frame, bbox)


class CustomSelectionTracker:
    def __init__(self, tracker_name: str, selection_file: Path) -> None:
        self._tracker_name = tracker_name
        self._selection_file = selection_file
        self._tracker: object | None = None
        self._selection_mtime_ns: int | None = None
        self._smoothed_bbox: tuple[int, int, int, int] | None = None
        self._misses = 0
        self._max_held_misses = 12

    def bbox(self, frame: object) -> tuple[int, int, int, int] | None:
        if not self._selection_file.exists():
            raise RuntimeError("custom selection is missing; select a target in the UI first")
        mtime_ns = self._selection_file.stat().st_mtime_ns
        if self._tracker is None or mtime_ns != self._selection_mtime_ns:
            initial_bbox = selection_bbox(self._selection_file, frame)
            tracker = create_manual_tracker(self._tracker_name)
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
            max_center_jump_norm=0.14,
            max_size_ratio=3.0,
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
            alpha=0.38,
            max_center_jump_norm=0.14,
            max_size_ratio=3.0,
        )
        return self._smoothed_bbox


class DetectorBackedTracker:
    """RDV-style detector initialization with OpenCV tracker updates."""

    def __init__(
        self,
        tracker_name: str,
        min_area: float,
        detector: Callable[
            [object, float],
            tuple[int, int, int, int] | None,
        ] = detect_colored_target,
    ) -> None:
        self._tracker_name = tracker_name
        self._min_area = min_area
        self._detector = detector
        self._tracker: object | None = None
        self._smoothed_bbox: tuple[int, int, int, int] | None = None
        self._misses = 0

    def bbox(self, frame: object) -> tuple[int, int, int, int] | None:
        frame_height, frame_width = frame.shape[:2]
        if self._tracker is None:
            return self._detect_and_initialize(frame)

        detector_bbox = self._detector(frame, self._min_area)
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
        detected_bbox = self._detector(frame, self._min_area)
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
    timeout_s = float(args.timeout_s) if args.timeout_s > 0 else 20.0
    send_client_heartbeat(connection)
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
    if not armed and not args.surface_test:
        raise RuntimeError("vehicle must already be armed; this script will not arm")

    return connection.target_system, connection.target_component, vehicle


def send_client_heartbeat(connection: mavutil.mavfile) -> None:
    connection.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        0,
    )


def heartbeat_status(message: object) -> tuple[bool, str]:
    armed = bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    return armed, mavutil.mode_string_v10(message)


def should_reassert_plane_mode(mode_name: str) -> bool:
    return mode_name == "RTL"


def poll_heartbeat(
    connection: mavutil.mavfile,
    last_timestamp_s: float,
) -> tuple[object | None, float]:
    try:
        message = connection.recv_match(type="HEARTBEAT", blocking=False, timeout=0)
    except (TypeError, OSError) as exc:
        print(f"sitl_tracking_status=heartbeat_skipped reason={type(exc).__name__}", flush=True)
        return None, last_timestamp_s
    if message is not None:
        return message, float(getattr(message, "_timestamp", time.time()))
    cached_messages = getattr(connection, "messages", {})
    cached = cached_messages.get("HEARTBEAT") if hasattr(cached_messages, "get") else None
    if cached is None:
        return None, last_timestamp_s
    cached_timestamp_s = float(getattr(cached, "_timestamp", 0.0) or 0.0)
    if cached_timestamp_s > last_timestamp_s:
        return cached, cached_timestamp_s
    return None, last_timestamp_s


def tracking_failsafe(reason: str, **fields: object) -> int:
    suffix = " ".join(f"{key}={value}" for key, value in fields.items())
    print(f"tracking_failsafe=active reason={reason}{(' ' + suffix) if suffix else ''}", flush=True)
    return 1


def safe_release_override(connection: mavutil.mavfile, active: bool, reason: str) -> bool:
    if not active:
        return False
    try:
        release_rc_override(connection)
    except (OSError, RuntimeError) as exc:
        print(
            f"tracking_failsafe=release_failed reason={reason} detail={type(exc).__name__}",
            flush=True,
        )
    else:
        print(f"tracking_control=released reason={reason}", flush=True)
    return False


def validate_rc_override_acceptance(
    values: dict[str, float],
    source_system: int,
) -> str | None:
    gcs_low = values.get("MAV_GCS_SYSID")
    gcs_high = values.get("MAV_GCS_SYSID_HI")
    if gcs_low is not None and gcs_high is not None:
        low = round(gcs_low)
        high = round(gcs_high)
        if high >= low and not (low <= source_system <= high):
            return "rc_override_sysid_mismatch"
        if high < low and low not in {0, source_system}:
            return "rc_override_sysid_mismatch"
    elif gcs_low is not None and round(gcs_low) not in {0, source_system}:
        return "rc_override_sysid_mismatch"
    return None


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
    roll_rc_reversed: bool = False,
    calibration: RcCalibrationSet | None = None,
) -> tuple[int, int, int, int]:
    roll_fraction = clamp(roll_deg / max(0.1, max_roll_deg), -1.0, 1.0)
    pitch_fraction = clamp(pitch_deg / max(0.1, max_pitch_deg), -1.0, 1.0)
    if roll_rc_reversed:
        roll_fraction = -roll_fraction
    if pitch_rc_reversed:
        pitch_fraction = -pitch_fraction
    throttle_fraction = clamp(throttle, 0.0, 1.0)
    calibration = calibration or RcCalibrationSet()
    return (
        pwm_from_centered_fraction(roll_fraction, calibration.roll),
        pwm_from_centered_fraction(pitch_fraction, calibration.pitch),
        pwm_from_throttle_fraction(throttle_fraction, calibration.throttle),
        calibration.yaw.trim,
    )


def pwm_from_centered_fraction(value: float, calibration: RcCalibration) -> int:
    value = clamp(-value if calibration.reversed else value, -1.0, 1.0)
    if value < 0.0:
        return round(calibration.trim + value * (calibration.trim - calibration.minimum))
    return round(calibration.trim + value * (calibration.maximum - calibration.trim))


def pwm_from_throttle_fraction(value: float, calibration: RcCalibration) -> int:
    value = 1.0 - value if calibration.reversed else value
    return round(
        calibration.minimum
        + clamp(value, 0.0, 1.0) * (calibration.maximum - calibration.minimum)
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
    error = clamp(error, -MAX_GUIDED_IMAGE_ERROR, MAX_GUIDED_IMAGE_ERROR)
    if previous_error is None or dt_s <= 0.0:
        return error
    previous_error = clamp(previous_error, -MAX_GUIDED_IMAGE_ERROR, MAX_GUIDED_IMAGE_ERROR)
    time_constant_s = max(0.0, damping_gain)
    if time_constant_s <= 0.0:
        return error
    alpha = clamp(dt_s / (time_constant_s + dt_s), 0.0, 1.0)
    return clamp(
        previous_error + (error - previous_error) * alpha,
        -MAX_GUIDED_IMAGE_ERROR,
        MAX_GUIDED_IMAGE_ERROR,
    )


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


def enforce_visible_surface_deflection(
    command_deg: float,
    error: float,
    min_deflection_deg: float,
    limit_deg: float,
    *,
    invert_error_sign: bool = False,
) -> float:
    min_deflection_deg = max(0.0, min(min_deflection_deg, abs(limit_deg)))
    if min_deflection_deg <= 0.0 or abs(error) <= 0.0:
        return command_deg
    direction = -math.copysign(1.0, error) if invert_error_sign else math.copysign(1.0, error)
    if command_deg * direction >= 0.0 and abs(command_deg) >= min_deflection_deg:
        return command_deg
    return clamp(direction * min_deflection_deg, -abs(limit_deg), abs(limit_deg))


def write_demand_state(path: Path | None, payload: dict[str, object]) -> None:
    if path is None:
        return
    payload = {**payload, "updated_monotonic_s": time.monotonic(), "updated_unix_s": time.time()}
    tmp_path = path.with_suffix(path.suffix + ".tmp")
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp_path.write_text(json.dumps(payload, sort_keys=True), encoding="utf-8")
        tmp_path.replace(path)
    except OSError:
        return


def send_plane_rc_attitude(
    connection: mavutil.mavfile,
    roll_deg: float,
    pitch_deg: float,
    max_roll_deg: float,
    max_pitch_deg: float,
    throttle: float,
    pitch_rc_reversed: bool = False,
    roll_channel: int = 1,
    pitch_channel: int = 2,
    throttle_channel: int = 3,
    yaw_channel: int = 4,
    roll_rc_reversed: bool = False,
    rc_calibration: RcCalibrationSet | None = None,
) -> None:
    roll_pwm, pitch_pwm, throttle_pwm, yaw_pwm = attitude_to_plane_rc_pwm(
        roll_deg,
        pitch_deg,
        max_roll_deg,
        max_pitch_deg,
        throttle,
        pitch_rc_reversed=pitch_rc_reversed,
        roll_rc_reversed=roll_rc_reversed,
        calibration=rc_calibration,
    )
    rc_override(
        connection,
        {
            roll_channel: roll_pwm,
            pitch_channel: pitch_pwm,
            throttle_channel: throttle_pwm,
            yaw_channel: yaw_pwm,
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


def load_plane_param_cache(path: Path | None, mavlink_endpoint: str) -> dict[str, float] | None:
    if path is None or not path.exists():
        return None
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    if payload.get("mavlink") != mavlink_endpoint:
        return None
    raw_values = payload.get("values")
    if not isinstance(raw_values, dict):
        return None
    values: dict[str, float] = {}
    for name, value in raw_values.items():
        if not isinstance(name, str):
            continue
        try:
            values[name] = float(value)
        except (TypeError, ValueError):
            continue
    return values if values else None


def save_plane_param_cache(
    path: Path | None,
    mavlink_endpoint: str,
    values: dict[str, float],
) -> None:
    if path is None or not values:
        return
    payload = {
        "mavlink": mavlink_endpoint,
        "updated_unix_s": time.time(),
        "values": values,
    }
    tmp_path = path.with_suffix(path.suffix + ".tmp")
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp_path.write_text(json.dumps(payload, sort_keys=True), encoding="utf-8")
        tmp_path.replace(path)
    except OSError:
        return


def plane_response_model_from_params(
    args: argparse.Namespace | PlaneTrackingTuning,
    values: dict[str, float],
) -> PlaneResponseModel:
    rate_hz = float(getattr(args, "rate_hz", 30.0))
    max_roll_deg = args.max_plane_roll_deg
    if "ROLL_LIMIT_DEG" in values and values["ROLL_LIMIT_DEG"] > 0:
        max_roll_deg = min(max_roll_deg, values["ROLL_LIMIT_DEG"])
    elif "LIM_ROLL_CD" in values and values["LIM_ROLL_CD"] > 0:
        max_roll_deg = min(max_roll_deg, values["LIM_ROLL_CD"] / 100.0)

    max_pitch_up_deg = args.max_plane_pitch_deg
    if "PTCH_LIM_MAX_DEG" in values and values["PTCH_LIM_MAX_DEG"] > 0:
        max_pitch_up_deg = min(max_pitch_up_deg, values["PTCH_LIM_MAX_DEG"])
    elif "LIM_PITCH_MAX" in values and values["LIM_PITCH_MAX"] > 0:
        max_pitch_up_deg = min(max_pitch_up_deg, values["LIM_PITCH_MAX"] / 100.0)

    max_pitch_down_deg = args.max_plane_pitch_deg
    if "PTCH_LIM_MIN_DEG" in values:
        max_pitch_down_deg = min(max_pitch_down_deg, abs(values["PTCH_LIM_MIN_DEG"]))
    elif "LIM_PITCH_MIN" in values:
        max_pitch_down_deg = min(max_pitch_down_deg, abs(values["LIM_PITCH_MIN"]) / 100.0)

    cruise_throttle = args.plane_throttle
    fixed_throttle = (
        abs(args.plane_throttle - args.plane_min_throttle) < 1e-6
        and abs(args.plane_throttle - args.plane_max_throttle) < 1e-6
    )
    if not fixed_throttle and "TRIM_THROTTLE" in values:
        cruise_throttle = clamp(values["TRIM_THROTTLE"] / 100.0, 0.0, 1.0)

    min_throttle = args.plane_min_throttle
    if not fixed_throttle and "THR_MIN" in values:
        min_throttle = clamp(values["THR_MIN"] / 100.0, 0.0, 1.0)

    max_throttle = args.plane_max_throttle
    if not fixed_throttle and "THR_MAX" in values:
        max_throttle = clamp(values["THR_MAX"] / 100.0, min_throttle, 1.0)

    target_airspeed_mps = args.plane_airspeed_mps
    if "ARSPD_FBW_MIN" in values:
        target_airspeed_mps = max(target_airspeed_mps, values["ARSPD_FBW_MIN"])
    if "ARSPD_FBW_MAX" in values and values["ARSPD_FBW_MAX"] > 0:
        target_airspeed_mps = min(target_airspeed_mps, values["ARSPD_FBW_MAX"])

    pitch_filter_alpha = args.plane_pitch_filter_alpha
    if "PTCH2SRV_TCONST" in values and values["PTCH2SRV_TCONST"] > 0:
        period_s = 1.0 / max(1.0, rate_hz)
        response_alpha = period_s / (values["PTCH2SRV_TCONST"] + period_s)
        pitch_filter_alpha = clamp(
            response_alpha,
            MIN_PLANE_COMMAND_FILTER_ALPHA,
            args.plane_pitch_filter_alpha,
        )

    max_pitch_step_deg = args.plane_max_pitch_step_deg
    if "PTCH2SRV_TCONST" in values and values["PTCH2SRV_TCONST"] > 0:
        max_pitch_step_deg = clamp(
            args.plane_max_pitch_step_deg / values["PTCH2SRV_TCONST"],
            1.0,
            args.plane_max_pitch_step_deg,
        )

    roll_channel = channel_from_param(values.get("RCMAP_ROLL", 1.0), 1)
    pitch_channel = channel_from_param(values.get("RCMAP_PITCH", 2.0), 2)
    throttle_channel = channel_from_param(values.get("RCMAP_THROTTLE", 3.0), 3)
    yaw_channel = channel_from_param(values.get("RCMAP_YAW", 4.0), 4)
    roll_rc_reversed = bool(
        f"RC{roll_channel}_REVERSED" in values
        and round(values[f"RC{roll_channel}_REVERSED"]) != 0
    )
    pitch_rc_reversed = bool(
        f"RC{pitch_channel}_REVERSED" in values
        and round(values[f"RC{pitch_channel}_REVERSED"]) != 0
    )
    rc_calibration = RcCalibrationSet(
        roll=rc_calibration_from_params(values, roll_channel, roll_rc_reversed),
        pitch=rc_calibration_from_params(values, pitch_channel, pitch_rc_reversed),
        throttle=rc_calibration_from_params(
            values,
            throttle_channel,
            bool(
                f"RC{throttle_channel}_REVERSED" in values
                and round(values[f"RC{throttle_channel}_REVERSED"]) != 0
            ),
        ),
        yaw=rc_calibration_from_params(
            values,
            yaw_channel,
            bool(
                f"RC{yaw_channel}_REVERSED" in values
                and round(values[f"RC{yaw_channel}_REVERSED"]) != 0
            ),
        ),
    )
    min_airspeed_mps = values.get("ARSPD_FBW_MIN")
    if min_airspeed_mps is not None and min_airspeed_mps <= 0:
        min_airspeed_mps = None
    rc_override_timeout_s = values.get("RC_OVERRIDE_TIME")
    if rc_override_timeout_s is not None and rc_override_timeout_s < 0:
        rc_override_timeout_s = None

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
        roll_rc_reversed=roll_rc_reversed,
        pitch_rc_reversed=pitch_rc_reversed,
        roll_channel=roll_channel,
        pitch_channel=pitch_channel,
        throttle_channel=throttle_channel,
        yaw_channel=yaw_channel,
        rc_calibration=rc_calibration,
        min_airspeed_mps=min_airspeed_mps,
        rc_override_timeout_s=rc_override_timeout_s,
        raw_params=values,
    )


def channel_from_param(value: float | None, default: int) -> int:
    if value is None:
        return default
    channel = round(value)
    if channel < 1 or channel > 8:
        return default
    return channel


def rc_calibration_from_params(
    values: dict[str, float],
    channel: int,
    reversed_: bool,
) -> RcCalibration:
    minimum = round(values.get(f"RC{channel}_MIN", 1000.0))
    trim = round(values.get(f"RC{channel}_TRIM", 1500.0))
    maximum = round(values.get(f"RC{channel}_MAX", 2000.0))
    if not (800 <= minimum < trim < maximum <= 2200):
        return RcCalibration(reversed=reversed_)
    return RcCalibration(minimum, trim, maximum, reversed_)


def format_servo_functions(values: dict[str, float]) -> str:
    items: list[str] = []
    for channel in range(1, 9):
        name = f"SERVO{channel}_FUNCTION"
        if name in values:
            items.append(f"{channel}:{round(values[name])}")
    return ",".join(items) if items else "unknown"


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
        try:
            message = connection.recv_match(
                type=["VFR_HUD", "GLOBAL_POSITION_INT"],
                blocking=True,
                timeout=0.005,
            )
        except (TypeError, OSError) as exc:
            print(f"sitl_tracking_status=telemetry_skipped reason={type(exc).__name__}", flush=True)
            return heading_deg, relative_alt_m, airspeed_mps
        if message is None:
            continue
        if message.get_type() == "VFR_HUD":
            heading_deg = float(message.heading)
            airspeed_mps = float(message.airspeed)
        elif message.get_type() == "GLOBAL_POSITION_INT":
            relative_alt_m = float(message.relative_alt) / 1000.0
    return heading_deg, relative_alt_m, airspeed_mps


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


class FrameDirectoryReader:
    def __init__(self, camera_dir: Path) -> None:
        self._camera_dir = camera_dir
        self._last_key: tuple[str, int, int] | None = None

    def read(self) -> tuple[bool, object | None, float | None]:
        image_path = newest_stable_image(self._camera_dir)
        if image_path is None:
            return False, None, None
        try:
            stat = image_path.stat()
        except FileNotFoundError:
            return False, None, None
        key = (str(image_path), stat.st_mtime_ns, stat.st_size)
        if key == self._last_key:
            return False, None, time.time() - stat.st_mtime
        frame = read_stable_image(image_path)
        if frame is None:
            return False, None, None
        self._last_key = key
        return True, frame, time.time() - stat.st_mtime


def newest_stable_image(camera_dir: Path, *, now: float | None = None) -> Path | None:
    now = time.time() if now is None else now
    candidates: list[tuple[float, Path]] = []
    for pattern in ("*.png", "*.jpg", "*.jpeg"):
        for path in camera_dir.glob(pattern):
            try:
                stat = path.stat()
            except FileNotFoundError:
                continue
            if not path.is_file() or stat.st_size <= 0:
                continue
            if now - stat.st_mtime < STABLE_FRAME_MIN_AGE_S:
                continue
            candidates.append((stat.st_mtime, path))
    if candidates:
        return max(candidates, key=lambda item: item[0])[1]
    return newest_image(camera_dir)


def read_stable_image(image_path: Path) -> object | None:
    last_size = -1
    for _ in range(3):
        try:
            size = image_path.stat().st_size
        except FileNotFoundError:
            return None
        if size <= 0:
            time.sleep(0.01)
            continue
        frame = cv2.imread(str(image_path))
        if frame is not None and size == last_size:
            return frame
        last_size = size
        time.sleep(0.01)
    return cv2.imread(str(image_path))


def main() -> int:
    args = parse_args()
    if not args.enable_guidance:
        print("sitl_tracking_status=failed reason=missing_--enable-guidance", flush=True)
        return 2
    plane_tuning = plane_tuning_from_args(args)
    tuning_file_state = TuningFileState(args.tuning_file)

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
    if args.surface_test and vehicle != "plane":
        print("sitl_tracking_status=failed reason=surface_test_requires_plane", flush=True)
        return 2
    surface_test_throttle = clamp(args.surface_test_throttle, 0.0, 0.80)
    if args.surface_test:
        print(
            "sitl_tracking_status=surface_test "
            f"vehicle=plane mode=fbwa throttle={surface_test_throttle:.2f}",
            flush=True,
        )

    capture = None if args.camera_dir else open_capture(args.pipeline or "")
    if capture is not None and not capture.isOpened():
        print("sitl_tracking_status=failed reason=camera_unavailable", flush=True)
        return 1
    frame_reader = FrameDirectoryReader(args.camera_dir) if args.camera_dir is not None else None

    custom_tracker = None
    tracking_mode = "red" if args.tracking_mode == "orange" else args.tracking_mode
    detector_tracker = None
    if tracking_mode in {"custom", "head", "person"}:
        if args.selection_file is None:
            print("sitl_tracking_status=failed reason=missing_custom_selection_file", flush=True)
            return 2
        custom_tracker = CustomSelectionTracker(args.tracker, args.selection_file)
    else:
        detector = detect_banner_target if tracking_mode == "banner" else detect_colored_target
        detector_tracker = DetectorBackedTracker(args.tracker, args.min_area, detector)

    period_s = 1.0 / args.rate_hz
    frames = 0
    detections = 0
    last_report = 0.0
    last_command = time.monotonic()
    last_valid_frame_monotonic = time.monotonic()
    last_heartbeat_timestamp_s = time.time()
    last_gcs_heartbeat_monotonic = time.monotonic()
    control_authority_active = False
    low_airspeed_since: float | None = None
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
    pitch_gain_report = plane_tuning.plane_pitch_gain_scale
    frame_age_s: float | None = None
    proximity_report = 0.0
    control_scale_report = 1.0
    centering_gain_report = plane_tuning.plane_centering_gain
    damping_gain_report = plane_tuning.plane_damping_gain
    airspeed_report: float | None = None
    roll_pwm_report = 1500
    pitch_pwm_report = 1500
    throttle_pwm_report = 1000
    plane_response_model = plane_response_model_from_params(plane_tuning, {})
    if vehicle == "plane":
        if args.read_plane_params:
            param_values = load_plane_param_cache(args.plane_param_cache_file, args.mavlink)
            param_source = "cache"
            if param_values is None:
                param_values = read_plane_parameters(
                    connection,
                    target_system,
                    target_component,
                    timeout_per_param_s=args.plane_param_timeout_s,
                )
                save_plane_param_cache(args.plane_param_cache_file, args.mavlink, param_values)
                param_source = "mavlink"
            block_reason = validate_rc_override_acceptance(param_values, args.source_system)
            if block_reason is not None:
                print(f"steering blocked reason={block_reason}", flush=True)
                return 1
            if "RC_OVERRIDE_TIME" not in param_values:
                print("tracking_warning=rc_override_timeout_unknown", flush=True)
            plane_response_model = plane_response_model_from_params(plane_tuning, param_values)
            param_status = "cached" if param_source == "cache" else "read"
            print(
                f"sitl_tracking_status=plane_params_{param_status} "
                f"count={len(param_values)} "
                f"max_roll_deg={plane_response_model.max_roll_deg:.2f} "
                f"max_pitch_up_deg={plane_response_model.max_pitch_up_deg:.2f} "
                f"max_pitch_down_deg={plane_response_model.max_pitch_down_deg:.2f} "
                f"target_airspeed_mps={plane_response_model.target_airspeed_mps:.2f} "
                f"pitch_filter_alpha={plane_response_model.pitch_filter_alpha:.2f} "
                f"max_pitch_step_deg={plane_response_model.max_pitch_step_deg:.2f} "
                f"rcmap_roll={plane_response_model.roll_channel} "
                f"rcmap_pitch={plane_response_model.pitch_channel} "
                f"rcmap_throttle={plane_response_model.throttle_channel} "
                f"rcmap_yaw={plane_response_model.yaw_channel} "
                f"pitch_rc_reversed={int(plane_response_model.pitch_rc_reversed)} "
                f"servo_functions={format_servo_functions(param_values)}",
                flush=True,
            )
        if args.surface_test:
            plane_response_model = plane_response_model._replace(
                pitch_filter_alpha=max(plane_response_model.pitch_filter_alpha, 0.55),
                max_pitch_step_deg=max(plane_response_model.max_pitch_step_deg, 6.0),
            )
            plane_tuning = plane_tuning._replace(
                plane_max_roll_step_deg=max(plane_tuning.plane_max_roll_step_deg, 8.0),
            )
        if not args.surface_test:
            plane_heading_deg, plane_relative_alt_m, plane_airspeed_mps = read_plane_telemetry(
                connection,
                plane_heading_deg,
                plane_relative_alt_m,
                plane_airspeed_mps,
            )
        if not args.surface_test:
            throttle_report = plane_throttle_for_airspeed(
                plane_airspeed_mps,
                plane_response_model.target_airspeed_mps,
                plane_response_model.cruise_throttle,
                plane_response_model.min_throttle,
                plane_response_model.max_throttle,
                plane_tuning.plane_throttle_airspeed_gain,
                0.0,
                max(plane_response_model.max_pitch_up_deg, plane_response_model.max_pitch_down_deg),
            )
        else:
            throttle_report = surface_test_throttle
        airspeed_report = plane_airspeed_mps
        send_plane_rc_attitude(
            connection,
            0.0,
            0.0,
            plane_response_model.max_roll_deg,
            max(plane_response_model.max_pitch_up_deg, plane_response_model.max_pitch_down_deg),
            throttle_report,
            plane_response_model.pitch_rc_reversed,
            plane_response_model.roll_channel,
            plane_response_model.pitch_channel,
            plane_response_model.throttle_channel,
            plane_response_model.yaw_channel,
            plane_response_model.roll_rc_reversed,
            plane_response_model.rc_calibration,
        )
        control_authority_active = True
        roll_pwm_report, pitch_pwm_report, throttle_pwm_report, _yaw_pwm = (
            attitude_to_plane_rc_pwm(
                0.0,
                0.0,
                plane_response_model.max_roll_deg,
                max(
                    plane_response_model.max_pitch_up_deg,
                    plane_response_model.max_pitch_down_deg,
                ),
                throttle_report,
                roll_rc_reversed=plane_response_model.roll_rc_reversed,
                pitch_rc_reversed=plane_response_model.pitch_rc_reversed,
                calibration=plane_response_model.rc_calibration,
            )
        )

    try:
        while running:
            now = time.monotonic()
            if now - last_gcs_heartbeat_monotonic >= 0.5:
                send_client_heartbeat(connection)
                last_gcs_heartbeat_monotonic = now
            heartbeat, last_heartbeat_timestamp_s = poll_heartbeat(
                connection,
                last_heartbeat_timestamp_s,
            )
            if heartbeat is not None:
                armed, current_mode = heartbeat_status(heartbeat)
                if vehicle == "plane":
                    if not armed and not args.surface_test:
                        control_authority_active = safe_release_override(
                            connection,
                            control_authority_active,
                            "vehicle_disarmed",
                        )
                        return tracking_failsafe("vehicle_disarmed")
                    if current_mode == "RTL":
                        fbwa_mode = connection.mode_mapping().get("FBWA")
                        if fbwa_mode is not None:
                            print(
                                "sitl_tracking_status=reasserting_mode "
                                f"vehicle=plane from={current_mode} to=FBWA",
                                flush=True,
                            )
                            connection.set_mode(fbwa_mode)
                            continue
                    if current_mode != "FBWA":
                        control_authority_active = safe_release_override(
                            connection,
                            control_authority_active,
                            "flight_mode_changed",
                        )
                        return tracking_failsafe("flight_mode_changed", mode=current_mode)
            if vehicle == "plane":
                tuning_file_state, plane_tuning = update_tuning_from_file(
                    tuning_file_state,
                    plane_tuning,
                )
                plane_response_model = plane_response_model_from_params(
                    plane_tuning,
                    plane_response_model.raw_params,
                )
            if frame_reader is not None:
                ok, frame, frame_age_s = frame_reader.read()
            else:
                ok, frame = frame_from_source(capture, args.camera_dir)
                frame_age_s = None
            if not ok or frame is None:
                if vehicle == "plane":
                    frame_age_ms = (time.monotonic() - last_valid_frame_monotonic) * 1000.0
                    if frame_age_ms > max(100.0, float(args.max_frame_age_ms)):
                        control_authority_active = safe_release_override(
                            connection,
                            control_authority_active,
                            "stale_video",
                        )
                        write_demand_state(
                            args.demand_state_file,
                            {
                                "detected": False,
                                "mode": tracking_mode,
                                "vehicle": vehicle,
                                "failsafe": True,
                                "failsafe_reason": "stale_video",
                                "frame_age_ms": frame_age_ms,
                                "tracking_tuning_revision": plane_tuning.revision,
                            },
                        )
                        return tracking_failsafe("stale_video", frame_age_ms=f"{frame_age_ms:.0f}")
                time.sleep(min(0.05, period_s / 2.0))
                continue

            frames += 1
            last_valid_frame_monotonic = time.monotonic()
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
                    hold_age_s = (
                        float("inf") if last_detection_time is None else now - last_detection_time
                    )
                    hold_attitude = (
                        previous_roll_command_deg is not None
                        and previous_pitch_command_deg is not None
                        and hold_age_s <= max(0.0, plane_tuning.plane_loss_hold_s)
                    )
                    if not hold_attitude:
                        if last_detection_time is not None:
                            control_authority_active = safe_release_override(
                                connection,
                                control_authority_active,
                                "target_lost",
                            )
                            write_demand_state(
                                args.demand_state_file,
                                {
                                    "detected": False,
                                    "mode": tracking_mode,
                                    "vehicle": vehicle,
                                    "failsafe": True,
                                    "failsafe_reason": "target_lost",
                                    "tracking_tuning_revision": plane_tuning.revision,
                                },
                            )
                            return tracking_failsafe("target_lost")
                        previous_roll_command_deg = 0.0
                        previous_pitch_command_deg = 0.0
                        if args.surface_test:
                            throttle_report = surface_test_throttle
                        else:
                            throttle_report = plane_throttle_for_airspeed(
                                plane_airspeed_mps,
                                plane_response_model.target_airspeed_mps,
                                plane_response_model.cruise_throttle,
                                plane_response_model.min_throttle,
                                plane_response_model.max_throttle,
                                plane_tuning.plane_throttle_airspeed_gain,
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
                        plane_response_model.roll_channel,
                        plane_response_model.pitch_channel,
                        plane_response_model.throttle_channel,
                        plane_response_model.yaw_channel,
                        plane_response_model.roll_rc_reversed,
                        plane_response_model.rc_calibration,
                    )
                    control_authority_active = True
                    (
                        roll_pwm_report,
                        pitch_pwm_report,
                        throttle_pwm_report,
                        _yaw_pwm,
                    ) = attitude_to_plane_rc_pwm(
                        previous_roll_command_deg,
                        previous_pitch_command_deg,
                        plane_response_model.max_roll_deg,
                        max(
                            plane_response_model.max_pitch_up_deg,
                            plane_response_model.max_pitch_down_deg,
                        ),
                        throttle_report,
                        pitch_rc_reversed=plane_response_model.pitch_rc_reversed,
                        roll_rc_reversed=plane_response_model.roll_rc_reversed,
                        calibration=plane_response_model.rc_calibration,
                    )
                    write_demand_state(
                        args.demand_state_file,
                        {
                            "detected": False,
                            "mode": tracking_mode,
                            "vehicle": vehicle,
                            "surface_test": args.surface_test,
                            "loss_behavior": "holding_last_attitude",
                            "roll_deg": previous_roll_command_deg,
                            "pitch_deg": previous_pitch_command_deg,
                            "roll_pwm": roll_pwm_report,
                            "pitch_pwm": pitch_pwm_report,
                            "throttle_pwm": throttle_pwm_report,
                            "max_roll_deg": plane_response_model.max_roll_deg,
                            "max_pitch_deg": max(
                                plane_response_model.max_pitch_up_deg,
                                plane_response_model.max_pitch_down_deg,
                            ),
                            "rate_hz": args.rate_hz,
                            "frame_age_ms": (
                                frame_age_s * 1000.0 if frame_age_s is not None else None
                            ),
                            "tracking_tuning_revision": plane_tuning.revision,
                            "failsafe": False,
                        },
                    )
                if now - last_report >= 1.0:
                    frame_age_ms = frame_age_s * 1000.0 if frame_age_s is not None else -1.0
                    loss_behavior = (
                        "holding_last_attitude"
                        if vehicle == "plane" and hold_attitude
                        else "neutral_search"
                    )
                    print(
                        f"sitl_tracking_status=searching mode={tracking_mode} "
                        "reason=target_not_detected "
                        f"loss_behavior={loss_behavior} "
                        f"loss_hold_s={plane_tuning.plane_loss_hold_s:.2f} "
                        f"frame_age_ms={frame_age_ms:.0f}",
                        flush=True,
                    )
                    last_report = now
                time.sleep(period_s)
                continue

            detections += 1
            last_detection_time = now
            x, y, width, height = bbox
            frame_height, frame_width = frame.shape[:2]
            target_proximity = target_proximity_from_bbox(
                bbox,
                frame_width,
                frame_height,
                far_size=plane_tuning.plane_proximity_far_size,
                near_size=plane_tuning.plane_proximity_near_size,
            )
            center_x = (x + width / 2) / frame_width
            center_y = (y + height / 2) / frame_height
            error_x = center_x - 0.5
            error_y = center_y - 0.5
            if vehicle == "plane":
                aspect_ratio = frame_width / frame_height
                vertical_fov_deg = math.degrees(
                    2.0
                    * math.atan(
                        math.tan(math.radians(plane_tuning.plane_camera_hfov_deg) * 0.5)
                        / aspect_ratio
                    )
                )
                guided_error_x = perspective_correct_error(
                    error_x,
                    plane_tuning.plane_camera_hfov_deg,
                )
                guided_error_y = perspective_correct_error(error_y, vertical_fov_deg)
            else:
                guided_error_x = error_x
                guided_error_y = error_y
            if vehicle == "plane":
                guided_error_x = apply_deadband(guided_error_x, plane_tuning.plane_error_deadband)
                guided_error_y = apply_deadband(guided_error_y, plane_tuning.plane_error_deadband)
            measured_error_x = guided_error_x
            measured_error_y = guided_error_y
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
                lead_s = max(0.0, plane_tuning.plane_lead_s)
                if lead_s > 0.0:
                    proportional_error_x += (
                        (proportional_error_x - previous_error_x) / dt
                    ) * lead_s
                    proportional_error_y += (
                        (proportional_error_y - previous_error_y) / dt
                    ) * lead_s
                    proportional_error_x = clamp(
                        proportional_error_x,
                        -MAX_GUIDED_IMAGE_ERROR,
                        MAX_GUIDED_IMAGE_ERROR,
                    )
                    proportional_error_y = clamp(
                        proportional_error_y,
                        -MAX_GUIDED_IMAGE_ERROR,
                        MAX_GUIDED_IMAGE_ERROR,
                    )
                damping_gain = scheduled_gain(
                    plane_tuning.plane_damping_gain,
                    plane_tuning.plane_near_damping_gain,
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
            if vehicle == "plane":
                previous_error_x = measured_error_x
                previous_error_y = measured_error_y
            previous_error_time = now

            right_mps = clamp(error_x * 0.8, -args.max_right_mps, args.max_right_mps)
            down_mps = clamp(
                error_y * plane_tuning.vertical_gain,
                -args.max_down_mps,
                args.max_down_mps,
            )
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
                if not args.surface_test:
                    (
                        plane_heading_deg,
                        plane_relative_alt_m,
                        plane_airspeed_mps,
                    ) = read_plane_telemetry(
                        connection,
                        plane_heading_deg,
                        plane_relative_alt_m,
                        plane_airspeed_mps,
                    )
                    if plane_relative_alt_m < plane_tuning.min_tracking_alt_m:
                        control_authority_active = safe_release_override(
                            connection,
                            control_authority_active,
                            "below_tracking_altitude",
                        )
                        return tracking_failsafe(
                            "below_tracking_altitude",
                            relative_alt_m=f"{plane_relative_alt_m:.1f}",
                            minimum_m=f"{plane_tuning.min_tracking_alt_m:.1f}",
                        )
                    if (
                        plane_response_model.min_airspeed_mps is not None
                        and plane_airspeed_mps is not None
                        and plane_airspeed_mps < plane_response_model.min_airspeed_mps
                    ):
                        if low_airspeed_since is None:
                            low_airspeed_since = now
                            print(
                                "tracking_warning=low_airspeed "
                                f"airspeed_mps={plane_airspeed_mps:.2f} "
                                f"minimum_mps={plane_response_model.min_airspeed_mps:.2f}",
                                flush=True,
                            )
                        elif now - low_airspeed_since >= plane_tuning.airspeed_low_persistence_s:
                            control_authority_active = safe_release_override(
                                connection,
                                control_authority_active,
                                "low_airspeed",
                            )
                            return tracking_failsafe(
                                "low_airspeed",
                                airspeed_mps=f"{plane_airspeed_mps:.2f}",
                                minimum_mps=f"{plane_response_model.min_airspeed_mps:.2f}",
                            )
                    else:
                        low_airspeed_since = None
                far_scale = far_target_control_scale(
                    target_proximity,
                    plane_tuning.plane_far_control_scale,
                )
                near_scale = near_target_control_scale(
                    target_proximity,
                    plane_tuning.plane_near_control_scale,
                )
                control_scale = far_scale * near_scale
                control_scale_report = control_scale
                centering_gain = scheduled_gain(
                    plane_tuning.plane_centering_gain,
                    plane_tuning.plane_near_centering_gain,
                    target_proximity,
                )
                centering_gain_report = centering_gain
                roll_deg = clamp(
                    guided_error_x
                    * plane_tuning.vertical_gain
                    * centering_gain
                    * plane_tuning.plane_roll_gain_scale
                    * control_scale,
                    -plane_response_model.max_roll_deg,
                    plane_response_model.max_roll_deg,
                )
                roll_deg = damp_pitch_command(
                    previous_roll_command_deg,
                    roll_deg,
                    plane_response_model.pitch_filter_alpha,
                    plane_tuning.plane_max_roll_step_deg,
                )
                if args.surface_test:
                    roll_deg = enforce_visible_surface_deflection(
                        roll_deg,
                        guided_error_x,
                        args.surface_test_min_deflection_deg,
                        plane_response_model.max_roll_deg,
                    )
                previous_roll_command_deg = roll_deg
                pitch_deg = plane_pitch_command(
                    guided_error_y,
                    plane_tuning.vertical_gain * centering_gain * control_scale,
                    plane_tuning.plane_pitch_gain_scale,
                    plane_tuning.plane_pitch_near_gain_scale,
                    target_proximity,
                    plane_tuning.plane_pitch_below_center_boost,
                    max(
                        plane_response_model.max_pitch_up_deg,
                        plane_response_model.max_pitch_down_deg,
                    ),
                    plane_tuning.plane_near_pitch_down_limit_deg,
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
                if args.surface_test:
                    pitch_deg = enforce_visible_surface_deflection(
                        pitch_deg,
                        guided_error_y,
                        args.surface_test_min_deflection_deg,
                        max(
                            plane_response_model.max_pitch_up_deg,
                            plane_response_model.max_pitch_down_deg,
                        ),
                        invert_error_sign=True,
                    )
                previous_pitch_command_deg = pitch_deg
                pitch_gain_report = adaptive_pitch_gain(
                    plane_tuning.plane_pitch_gain_scale,
                    plane_tuning.plane_pitch_near_gain_scale,
                    target_proximity,
                )
                proximity_report = target_proximity
                if args.surface_test:
                    throttle_report = surface_test_throttle
                else:
                    throttle_report = plane_throttle_for_airspeed(
                        plane_airspeed_mps,
                        plane_response_model.target_airspeed_mps,
                        plane_response_model.cruise_throttle,
                        plane_response_model.min_throttle,
                        plane_response_model.max_throttle,
                        plane_tuning.plane_throttle_airspeed_gain,
                        pitch_deg,
                        max(
                            plane_response_model.max_pitch_up_deg,
                            plane_response_model.max_pitch_down_deg,
                        ),
                        target_proximity,
                        plane_tuning.plane_near_throttle_reduction,
                    )
                airspeed_report = plane_airspeed_mps
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
                    plane_response_model.roll_channel,
                    plane_response_model.pitch_channel,
                    plane_response_model.throttle_channel,
                    plane_response_model.yaw_channel,
                    plane_response_model.roll_rc_reversed,
                    plane_response_model.rc_calibration,
                )
                control_authority_active = True
                (
                    roll_pwm_report,
                    pitch_pwm_report,
                    throttle_pwm_report,
                    _yaw_pwm,
                ) = attitude_to_plane_rc_pwm(
                    roll_deg,
                    pitch_deg,
                    plane_response_model.max_roll_deg,
                    max(
                        plane_response_model.max_pitch_up_deg,
                        plane_response_model.max_pitch_down_deg,
                    ),
                    throttle_report,
                    roll_rc_reversed=plane_response_model.roll_rc_reversed,
                    pitch_rc_reversed=plane_response_model.pitch_rc_reversed,
                    calibration=plane_response_model.rc_calibration,
                )
                write_demand_state(
                    args.demand_state_file,
                    {
                        "detected": True,
                        "mode": tracking_mode,
                        "vehicle": vehicle,
                        "surface_test": args.surface_test,
                        "bbox": [x, y, width, height],
                        "center_x": center_x,
                        "center_y": center_y,
                        "guided_error_x": guided_error_x,
                        "guided_error_y": guided_error_y,
                        "roll_deg": roll_deg,
                        "pitch_deg": pitch_deg,
                        "roll_pwm": roll_pwm_report,
                        "pitch_pwm": pitch_pwm_report,
                        "throttle_pwm": throttle_pwm_report,
                        "max_roll_deg": plane_response_model.max_roll_deg,
                        "max_pitch_deg": max(
                            plane_response_model.max_pitch_up_deg,
                            plane_response_model.max_pitch_down_deg,
                        ),
                        "rate_hz": args.rate_hz,
                        "frame_age_ms": frame_age_s * 1000.0 if frame_age_s is not None else None,
                        "target_proximity": target_proximity,
                        "throttle": throttle_report,
                        "airspeed_mps": airspeed_report,
                        "effective_max_roll_deg": plane_response_model.max_roll_deg,
                        "effective_max_pitch_deg": max(
                            plane_response_model.max_pitch_up_deg,
                            plane_response_model.max_pitch_down_deg,
                        ),
                        "tracking_tuning_revision": plane_tuning.revision,
                        "failsafe": False,
                    },
                )
            last_command = now

            if now - last_report >= 1.0:
                frame_age_ms = frame_age_s * 1000.0 if frame_age_s is not None else -1.0
                print(
                    "sitl_tracking_status=commanding "
                    f"vehicle={vehicle} "
                    f"mode={tracking_mode} "
                    f"center={center_x:.3f},{center_y:.3f} "
                    f"vel_body_frd={forward_mps:.2f},{right_mps:.2f},{down_mps:.2f} "
                    f"yaw_rate_deg_s={yaw_rate:.2f} "
                    f"roll_deg={roll_report:.2f} "
                    f"pitch_deg={pitch_report:.2f} "
                    f"roll_pwm={roll_pwm_report} "
                    f"pitch_pwm={pitch_pwm_report} "
                    f"throttle_pwm={throttle_pwm_report} "
                    f"rcmap={plane_response_model.roll_channel},"
                    f"{plane_response_model.pitch_channel},"
                    f"{plane_response_model.throttle_channel},"
                    f"{plane_response_model.yaw_channel} "
                    f"pitch_rc_reversed={int(plane_response_model.pitch_rc_reversed)} "
                    f"guided_error={guided_error_x:.3f},{guided_error_y:.3f} "
                    f"target_proximity={proximity_report:.2f} "
                    f"near_control_scale={control_scale_report:.2f} "
                    f"centering_gain={centering_gain_report:.2f} "
                    f"damping_gain={damping_gain_report:.2f} "
                    f"pitch_gain={pitch_gain_report:.2f} "
                    f"pitch_below_boost={plane_tuning.plane_pitch_below_center_boost:.2f} "
                    f"near_pitch_down_limit_deg={plane_tuning.plane_near_pitch_down_limit_deg:.2f} "
                    f"near_throttle_reduction={plane_tuning.plane_near_throttle_reduction:.2f} "
                    f"pitch_filter_alpha={plane_response_model.pitch_filter_alpha:.2f} "
                    f"pitch_step_deg={plane_response_model.max_pitch_step_deg:.2f} "
                    f"surface_min_deflection_deg={args.surface_test_min_deflection_deg:.2f} "
                    f"frame_age_ms={frame_age_ms:.0f} "
                    f"airspeed_mps={airspeed_report if airspeed_report is not None else -1.0:.2f} "
                    f"target_airspeed_mps={plane_response_model.target_airspeed_mps:.2f} "
                    f"throttle={throttle_report:.2f} "
                    f"surface_test={int(args.surface_test)} "
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
            control_authority_active = safe_release_override(
                connection,
                control_authority_active,
                "shutdown",
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
