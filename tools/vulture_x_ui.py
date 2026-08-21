#!/usr/bin/env python3
# ruff: noqa: E501
"""Local web control panel for Vulture-X SITL/Gazebo testing."""

from __future__ import annotations

import argparse
import contextlib
import json
import math
import os
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
import webbrowser
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, NamedTuple
from urllib.parse import parse_qs, urlparse

import cv2
import sitl_track_target as stt
from mavlink_endpoint import open_mavlink_connection, parse_mavlink_endpoint
from pymavlink import mavutil
from track_camera_target import (
    detect_banner_target,
    detect_colored_target,
    detect_heads,
    newest_image,
)

from vulture_x.runtime import TrackingRuntime
from vulture_x.runtime.capture import (
    FileFrameSource,
    GstAppSinkFrameSource,
    appsink_pipeline_from_rtsp,
    appsink_pipeline_from_udp_h264,
)
from vulture_x.runtime.mavlink import MavlinkTelemetryReceiver
from vulture_x.runtime.preview import encode_runtime_preview_jpeg
from vulture_x.runtime.state import RuntimeSnapshot, TrackingSnapshot, VideoFrame
from vulture_x.runtime.workers import ControlInputs
from vulture_x.vision.tracker import (
    TemplateMatchingTracker,
    clamp_bbox,
    refine_bbox_to_salient_region,
)

REPO_ROOT = Path(__file__).resolve().parents[1]
LOG_DIR = REPO_ROOT / "logs" / "ui"
CAMERA_DIR = LOG_DIR / "camera_frames"
SELECTION_PATH = LOG_DIR / "custom_selection.json"
DEMAND_STATE_PATH = LOG_DIR / "tracking_demand.json"
TRACKING_TUNING_PATH = LOG_DIR / "tracking_tuning.json"
PLANE_PARAM_CACHE_PATH = LOG_DIR / "plane_params_cache.json"
HEAD_DETECTION_MAX_WIDTH = 420
DEFAULT_NANO_BACKBONE = REPO_ROOT / "models" / "opencv" / "nanotrack" / "nanotrack_backbone_sim.onnx"
DEFAULT_NANO_NECKHEAD = REPO_ROOT / "models" / "opencv" / "nanotrack" / "nanotrack_head_sim.onnx"
DEFAULT_YUNET_MODEL = REPO_ROOT / "models" / "opencv" / "yunet" / "face_detection_yunet_2023mar.onnx"
SIM_MAVLINK_ENDPOINT = os.environ.get(
    "VULTURE_X_SIM_MAVLINK",
    os.environ.get("VULTURE_X_MAVLINK", "udpin:0.0.0.0:14550"),
)
MANUAL_MAVLINK_ENDPOINT = os.environ.get(
    "VULTURE_X_MANUAL_MAVLINK",
    os.environ.get("VULTURE_X_MAVLINK", "udpcl:192.168.144.12:19856"),
)
DEFAULT_MAVLINK_ENDPOINT = MANUAL_MAVLINK_ENDPOINT
MAVLINK_STATUS_ENDPOINT = os.environ.get("VULTURE_X_UI_MAVLINK", "udpin:0.0.0.0:14552")
SIM_VIDEO_SOURCE = os.environ.get("VULTURE_X_SIM_VIDEO_SOURCE", "udp")
MANUAL_VIDEO_SOURCE = os.environ.get("VULTURE_X_MANUAL_VIDEO_SOURCE", "rtsp")
DEFAULT_VIDEO_SOURCE = os.environ.get("VULTURE_X_VIDEO_SOURCE", MANUAL_VIDEO_SOURCE)
DEFAULT_RTSP_URL = os.environ.get("VULTURE_X_RTSP_URL", "rtsp://192.168.144.25:8554/main.264")
DEFAULT_RTSP_LATENCY_MS = os.environ.get("VULTURE_X_RTSP_LATENCY_MS", "50")
DEFAULT_RTSP_PROTOCOLS = os.environ.get("VULTURE_X_RTSP_PROTOCOLS", "tcp")
DEFAULT_RTSP_MAX_RATE = os.environ.get("VULTURE_X_RTSP_MAX_RATE", "30")
MAVLINK_STALE_AFTER_S = 3.5
FRESH_FRAME_MAX_AGE_S = 3.0
STABLE_FRAME_MIN_AGE_S = 0.02
MAX_GUIDED_FORWARD_MPS = 5.0
MAX_GUIDED_VERTICAL_MPS = 5.0
MAX_FIXED_WING_FORWARD_MPS = 20.0
MAX_FIXED_WING_VERTICAL_MPS = 10.0
MAX_FIXED_WING_VERTICAL_GAIN = 80.0
MAX_FIXED_WING_VERTICAL_LOOKAHEAD_S = 8.0
MIN_FIXED_WING_RELATIVE_ALT_M = 15.0
MAX_FIXED_WING_PITCH_DEG = 40.0
MAX_FIXED_WING_ROLL_DEG = 35.0
DEFAULT_FIXED_WING_VERTICAL_GAIN = 52.0
DEFAULT_FIXED_WING_CENTERING_GAIN = 1.55
DEFAULT_FIXED_WING_NEAR_CENTERING_GAIN = 2.65
DEFAULT_FIXED_WING_DAMPING_GAIN = 0.14
DEFAULT_FIXED_WING_NEAR_DAMPING_GAIN = 0.30
DEFAULT_FIXED_WING_PITCH_GAIN_SCALE = 1.20
DEFAULT_FIXED_WING_PITCH_NEAR_GAIN_SCALE = 1.60
DEFAULT_FIXED_WING_FAR_CONTROL_SCALE = 0.72
DEFAULT_FIXED_WING_PITCH_BELOW_BOOST = 0.25
DEFAULT_FIXED_WING_PITCH_FILTER_ALPHA = 0.45
DEFAULT_FIXED_WING_MAX_PITCH_STEP_DEG = 2.0
DEFAULT_FIXED_WING_MAX_ROLL_STEP_DEG = 6.0
DEFAULT_FIXED_WING_LOSS_HOLD_S = 1.5
DEFAULT_SIM_TRACKING_THROTTLE = 0.80
DEFAULT_GROUND_TEST_THROTTLE = 0.0
DEFAULT_MAX_FRAME_AGE_MS = 750.0
DEFAULT_MAVLINK_TIMEOUT_MS = 3000.0
CAMERA_STREAM_ENABLE_RETRY_S = (0.0, 1.0, 2.5, 5.0, 8.0)
CAMERA_START_WAIT_S = 5.0
HEAD_DETECTION_CACHE: dict[str, object] = {"key": None, "heads": []}
YUNET_INPUT_SIZE = (320, 320)
YUNET_SCORE_THRESHOLD = 0.85


class VehicleProfile(NamedTuple):
    mode: str
    label: str
    target_world: str
    target_motion_args: tuple[str, ...]
    autostart_target_motion: bool
    gazebo_arg: str
    sitl_arg: str
    sitl_process_pattern: str
    gazebo_process_pattern: str
    steering_supported: bool
    max_forward_mps: float
    max_vertical_mps: float
    max_vertical_gain: float
    max_vertical_lookahead_s: float
    min_relative_alt_m: float
    max_pitch_deg: float
    max_roll_deg: float


VEHICLE_PROFILES = {
    "quad": VehicleProfile(
        mode="quad",
        label="Quadcopter",
        target_world="vulture_x_test",
        target_motion_args=(),
        autostart_target_motion=True,
        gazebo_arg="-quad",
        sitl_arg="-quad",
        sitl_process_pattern=r"arducopter --model JSON",
        gazebo_process_pattern=r"gz sim .*vulture_x_test",
        steering_supported=True,
        max_forward_mps=MAX_GUIDED_FORWARD_MPS,
        max_vertical_mps=MAX_GUIDED_VERTICAL_MPS,
        max_vertical_gain=8.0,
        max_vertical_lookahead_s=0.0,
        min_relative_alt_m=0.0,
        max_pitch_deg=0.0,
        max_roll_deg=0.0,
    ),
    "plane": VehicleProfile(
        mode="plane",
        label="Fixed wing",
        target_world="vulture_x_plane",
        target_motion_args=(
            "--center-x",
            "32",
            "--center-y",
            "-16",
            "--center-z",
            "5.2",
            "--pitch-deg",
            "8",
            "--fixed-yaw-deg",
            "-25",
        ),
        autostart_target_motion=False,
        gazebo_arg="-plane",
        sitl_arg="-plane",
        sitl_process_pattern=r"arduplane --model JSON",
        gazebo_process_pattern=r"gz sim .*vulture_x_plane",
        steering_supported=True,
        max_forward_mps=MAX_FIXED_WING_FORWARD_MPS,
        max_vertical_mps=MAX_FIXED_WING_VERTICAL_MPS,
        max_vertical_gain=MAX_FIXED_WING_VERTICAL_GAIN,
        max_vertical_lookahead_s=MAX_FIXED_WING_VERTICAL_LOOKAHEAD_S,
        min_relative_alt_m=MIN_FIXED_WING_RELATIVE_ALT_M,
        max_pitch_deg=MAX_FIXED_WING_PITCH_DEG,
        max_roll_deg=MAX_FIXED_WING_ROLL_DEG,
    ),
}
DEFAULT_VEHICLE_MODE = "quad"

PLANE_TUNING_DEFAULTS: dict[str, float] = {
    "plane_airspeed_mps": 20.0,
    "vertical_gain": DEFAULT_FIXED_WING_VERTICAL_GAIN,
    "plane_centering_gain": DEFAULT_FIXED_WING_CENTERING_GAIN,
    "plane_near_centering_gain": DEFAULT_FIXED_WING_NEAR_CENTERING_GAIN,
    "plane_roll_gain_scale": 1.75,
    "plane_pitch_gain_scale": DEFAULT_FIXED_WING_PITCH_GAIN_SCALE,
    "plane_pitch_near_gain_scale": DEFAULT_FIXED_WING_PITCH_NEAR_GAIN_SCALE,
    "plane_error_deadband": 0.015,
    "plane_lead_s": 0.0,
    "plane_damping_gain": DEFAULT_FIXED_WING_DAMPING_GAIN,
    "plane_near_damping_gain": DEFAULT_FIXED_WING_NEAR_DAMPING_GAIN,
    "plane_pitch_filter_alpha": DEFAULT_FIXED_WING_PITCH_FILTER_ALPHA,
    "plane_max_pitch_step_deg": DEFAULT_FIXED_WING_MAX_PITCH_STEP_DEG,
    "plane_max_roll_step_deg": DEFAULT_FIXED_WING_MAX_ROLL_STEP_DEG,
    "max_plane_roll_deg": MAX_FIXED_WING_ROLL_DEG,
    "max_plane_pitch_deg": MAX_FIXED_WING_PITCH_DEG,
    "plane_near_pitch_down_limit_deg": MAX_FIXED_WING_PITCH_DEG,
    "plane_far_control_scale": DEFAULT_FIXED_WING_FAR_CONTROL_SCALE,
    "plane_near_control_scale": 1.0,
    "plane_camera_hfov_deg": 70.0,
    "plane_proximity_far_size": 0.025,
    "plane_proximity_near_size": 0.16,
    "plane_throttle": 0.55,
    "plane_throttle_airspeed_gain": 0.04,
    "plane_min_throttle": 0.25,
    "plane_max_throttle": 0.80,
    "plane_near_throttle_reduction": 0.0,
    "plane_pitch_below_center_boost": DEFAULT_FIXED_WING_PITCH_BELOW_BOOST,
    "plane_loss_hold_s": DEFAULT_FIXED_WING_LOSS_HOLD_S,
    "min_tracking_alt_m": MIN_FIXED_WING_RELATIVE_ALT_M,
    "airspeed_low_persistence_s": 2.0,
}

PLANE_TUNING_RANGES: dict[str, tuple[float, float]] = {
    "plane_airspeed_mps": (5.0, 40.0),
    "vertical_gain": (0.0, MAX_FIXED_WING_VERTICAL_GAIN),
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
    "plane_max_pitch_step_deg": (1.0, 10.0),
    "plane_max_roll_step_deg": (1.0, 10.0),
    "max_plane_roll_deg": (1.0, MAX_FIXED_WING_ROLL_DEG),
    "max_plane_pitch_deg": (1.0, MAX_FIXED_WING_PITCH_DEG),
    "plane_near_pitch_down_limit_deg": (0.0, MAX_FIXED_WING_PITCH_DEG),
    "plane_far_control_scale": (0.1, 1.0),
    "plane_near_control_scale": (0.1, 1.0),
    "plane_camera_hfov_deg": (20.0, 140.0),
    "plane_proximity_far_size": (0.001, 0.5),
    "plane_proximity_near_size": (0.002, 0.8),
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


def clamp_plane_tuning_values(values: dict[str, object]) -> dict[str, float]:
    result = dict(PLANE_TUNING_DEFAULTS)
    for key, raw_value in values.items():
        if key not in PLANE_TUNING_RANGES:
            continue
        result[key] = float(raw_value)
    for key, (lower, upper) in PLANE_TUNING_RANGES.items():
        result[key] = max(lower, min(upper, result[key]))
    if result["plane_proximity_near_size"] <= result["plane_proximity_far_size"]:
        result["plane_proximity_near_size"] = min(0.8, result["plane_proximity_far_size"] + 0.001)
    if result["plane_max_throttle"] < result["plane_min_throttle"]:
        result["plane_max_throttle"] = result["plane_min_throttle"]
    return result


def current_tuning_payload() -> dict[str, object]:
    try:
        payload = json.loads(TRACKING_TUNING_PATH.read_text(encoding="utf-8"))
    except (FileNotFoundError, OSError, json.JSONDecodeError):
        return {"revision": 0, "values": dict(PLANE_TUNING_DEFAULTS)}
    if not isinstance(payload, dict) or not isinstance(payload.get("values"), dict):
        return {"revision": 0, "values": dict(PLANE_TUNING_DEFAULTS)}
    return {
        "revision": int(payload.get("revision", 0)),
        "values": clamp_plane_tuning_values(payload["values"]),
    }


def write_tracking_tuning(values: dict[str, object]) -> dict[str, object]:
    current = current_tuning_payload()
    applied = clamp_plane_tuning_values({**current["values"], **values})
    revision = int(current["revision"]) + 1
    payload = {"ok": True, "revision": revision, "values": applied}
    tmp_path = TRACKING_TUNING_PATH.with_suffix(TRACKING_TUNING_PATH.suffix + ".tmp")
    TRACKING_TUNING_PATH.parent.mkdir(parents=True, exist_ok=True)
    tmp_path.write_text(json.dumps(payload, sort_keys=True), encoding="utf-8")
    tmp_path.replace(TRACKING_TUNING_PATH)
    return payload


class MavlinkStatusMonitor:
    def __init__(self, endpoint: str) -> None:
        self.endpoint = endpoint
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._status: dict[str, Any] = {
            "connected": False,
            "armed": False,
            "mode": "offline",
            "detail": "monitor not started",
            "endpoint": endpoint,
        }

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="mavlink-status", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            status = dict(self._status)
        last_seen = status.get("last_seen_monotonic_s")
        if isinstance(last_seen, float):
            age_s = time.monotonic() - last_seen
            status["age_s"] = round(age_s, 2)
            if age_s > MAVLINK_STALE_AFTER_S:
                status.update(
                    {
                        "connected": False,
                        "armed": False,
                        "mode": "offline",
                        "detail": f"heartbeat stale age_s={age_s:.1f}",
                    }
                )
        return status

    def _set_status(self, status: dict[str, Any]) -> None:
        status["endpoint"] = self.endpoint
        with self._lock:
            self._status = status

    def _run(self) -> None:
        while not self._stop.is_set():
            connection = None
            try:
                connection = open_mavlink_connection(
                    self.endpoint,
                    source_system=201,
                    source_component=202,
                    autoreconnect=False,
                )
                while not self._stop.is_set():
                    heartbeat = connection.recv_match(
                        type="HEARTBEAT",
                        blocking=True,
                        timeout=1.0,
                    )
                    if heartbeat is None:
                        continue
                    armed = bool(
                        heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    )
                    self._set_status(
                        {
                            "connected": True,
                            "armed": armed,
                            "mode": mavutil.mode_string_v10(heartbeat),
                            "type": int(heartbeat.type),
                            "autopilot": int(heartbeat.autopilot),
                            "last_seen_monotonic_s": time.monotonic(),
                        }
                    )
            except Exception as exc:
                self._set_status(
                    {
                        "connected": False,
                        "armed": False,
                        "mode": "offline",
                        "detail": str(exc),
                    }
                )
                self._stop.wait(1.0)
            finally:
                if connection is not None:
                    connection.close()

HTML = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Vulture-X SITL Panel</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #111314;
      --panel: #1b2022;
      --panel-2: #22292c;
      --text: #eef2ef;
      --muted: #9ba8a2;
      --line: #354044;
      --good: #60d394;
      --warn: #eec643;
      --bad: #ff6b6b;
      --accent: #69b7ff;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font: 15px/1.45 system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: var(--bg);
      color: var(--text);
    }
    header {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 16px;
      padding: 18px 22px;
      border-bottom: 1px solid var(--line);
      background: #15191b;
    }
    h1 { margin: 0; font-size: 20px; font-weight: 700; }
    main {
      display: grid;
      grid-template-columns: minmax(360px, 500px) minmax(420px, 1fr);
      gap: 18px;
      padding: 18px;
    }
    section {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 14px;
    }
    h2 { margin: 0 0 12px; font-size: 15px; }
    .status-grid { display: grid; gap: 8px; }
    .row {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
      min-height: 32px;
      border-bottom: 1px solid #2a3336;
    }
    .row:last-child { border-bottom: 0; }
    .muted { color: var(--muted); }
    .pill {
      min-width: 82px;
      text-align: center;
      padding: 4px 8px;
      border-radius: 999px;
      background: var(--panel-2);
      color: var(--muted);
      font-size: 12px;
      font-weight: 700;
    }
    .ok { color: var(--good); }
    .bad { color: var(--bad); }
    .warn { color: var(--warn); }
    .controls {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px;
      margin-top: 14px;
    }
    .top-actions {
      display: grid;
      grid-template-columns: 1.2fr 1.2fr 1fr 1fr;
      gap: 10px;
      margin-bottom: 18px;
    }
    .stack { display: grid; gap: 18px; }
    .connection-grid {
      display: grid;
      grid-template-columns: 1fr auto;
      gap: 10px;
      align-items: end;
      margin-bottom: 12px;
    }
    .connection-grid label { min-width: 0; }
    .connection-grid button { min-width: 132px; }
    .environment-controls {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 10px;
      margin-top: 14px;
    }
    button, input, select {
      height: 38px;
      border: 1px solid var(--line);
      border-radius: 6px;
      background: var(--panel-2);
      color: var(--text);
      font: inherit;
    }
    button {
      cursor: pointer;
      font-weight: 700;
    }
    button.primary {
      background: #0f5d56;
      border-color: #168678;
    }
    button.secondary {
      background: #17384a;
      border-color: #27627e;
    }
    button.danger {
      background: #5d1f26;
      border-color: #8f333d;
    }
    button:disabled {
      cursor: wait;
      opacity: 0.6;
    }
    label {
      display: grid;
      gap: 6px;
      color: var(--muted);
      font-size: 12px;
      font-weight: 700;
    }
    input, select { padding: 0 10px; }
    .inline-fields {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px;
      margin-bottom: 10px;
    }
    .tuning-grid { display: grid; gap: 12px; }
    details.tuning-group {
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 10px;
      background: #181d1f;
    }
    details.tuning-group > summary {
      cursor: pointer;
      font-weight: 800;
      margin-bottom: 8px;
    }
    .tuning-field {
      display: grid;
      grid-template-columns: 160px minmax(110px, 1fr) 82px;
      gap: 10px;
      align-items: center;
      padding: 8px 0;
      border-top: 1px solid #2a3336;
    }
    .tuning-field:first-of-type { border-top: 0; }
    .tuning-field small {
      grid-column: 1 / -1;
      color: var(--muted);
      margin-top: -4px;
    }
    .tuning-field input[type="range"] { width: 100%; padding: 0; }
    .tuning-actions {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 10px;
      margin-top: 12px;
    }
    .tuning-live {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 6px 14px;
      font-size: 13px;
      color: var(--muted);
    }
    .graph-panel canvas {
      width: 100%;
      height: 160px;
      border: 1px solid var(--line);
      border-radius: 8px;
      background: #101315;
    }
    .camera-wrap {
      display: grid;
      gap: 10px;
    }
    #camera {
      width: 100%;
      max-height: calc(100vh - 235px);
      aspect-ratio: 4 / 3;
      object-fit: contain;
      background: #070808;
      border: 1px solid var(--line);
      border-radius: 6px;
      display: block;
      user-select: none;
    }
    .camera-stage {
      position: relative;
      width: 100%;
    }
    #selection-box {
      position: absolute;
      display: none;
      border: 2px solid var(--accent);
      background: rgb(105 183 255 / 0.16);
      pointer-events: none;
    }
    .mode-row {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 8px;
      margin-bottom: 10px;
    }
    pre {
      height: 190px;
      overflow: auto;
      margin: 0;
      padding: 10px;
      background: #0b0d0e;
      border: 1px solid var(--line);
      border-radius: 6px;
      color: #cfd8d4;
      font-size: 12px;
      white-space: pre-wrap;
    }
    .two { display: grid; grid-template-columns: 1fr 1fr; gap: 18px; }
    .vehicle-fields[hidden] { display: none; }
    .field-title {
      margin: 12px 0 8px;
      color: var(--muted);
      font-weight: 700;
      font-size: 12px;
      text-transform: uppercase;
    }
    @media (max-width: 900px) {
      main, .two, .top-actions, .connection-grid, .environment-controls {
        grid-template-columns: 1fr;
      }
      #camera { max-height: none; }
    }
  </style>
</head>
<body>
  <header>
    <h1>Vulture-X SITL Panel</h1>
    <div class="muted">Local simulation controls</div>
  </header>
  <main>
    <div class="stack">
      <section>
        <h2>Quick Actions</h2>
        <div class="top-actions">
          <button class="primary" onclick="connectVideo()">Connect Video</button>
          <button id="tracking-btn" class="primary" onclick="steer()">
            Start Tracking (thr 80)
          </button>
          <button id="ground-test-btn" class="secondary" onclick="groundTest()">
            Ground Test (thr 0)
          </button>
          <button class="danger" onclick="post('/api/stop_steering')">Stop Steering</button>
        </div>
        <div id="sim-actions" class="environment-controls" hidden>
          <button onclick="post('/api/start_gazebo')">Start Gazebo</button>
          <button onclick="post('/api/start_sitl')">Start SITL</button>
          <button onclick="connectMavlink()">Connect MAVLink</button>
          <button
            id="takeoff-btn"
            class="secondary"
            onclick="post('/api/start_takeoff?mavlink=' + mavlinkEndpoint())"
          >
            Plane Takeoff
          </button>
          <button onclick="post('/api/start_target_motion')">Start Target</button>
          <button onclick="post('/api/stop_target_motion')">Stop Target</button>
          <button onclick="post('/api/clear_camera_cache')">Clear Camera</button>
        </div>
      </section>
      <section>
        <h2>Status</h2>
        <div class="status-grid" id="status"></div>
      </section>
      <section>
        <h2>Runtime Performance</h2>
        <div class="status-grid" id="runtime-performance"></div>
      </section>
      <section>
        <h2>Connections</h2>
        <div class="connection-grid">
          <label>Control MAVLink endpoint
            <input id="mavlink-endpoint" value="udpcl:192.168.144.12:19856">
          </label>
        </div>
        <label id="rtsp-url-label">RTSP URL
          <input id="rtsp-url" value="rtsp://192.168.144.25:8554/main.264">
        </label>
      </section>
      <section>
        <h2>Target</h2>
        <div class="mode-row">
          <label>Tracking
            <select id="tracking-mode">
              <option value="red" selected>Red object</option>
              <option value="head">Head/Face</option>
              <option value="custom">Manual box</option>
              <option value="banner">Sim banner</option>
            </select>
          </label>
          <label>Selection
            <input id="selection-status" value="none" readonly>
          </label>
        </div>
        <div id="quad-fields" class="vehicle-fields">
          <div class="field-title">Quad Guidance</div>
          <div class="inline-fields">
            <label>Forward speed m/s
              <input id="quad-forward-speed" type="number" min="0" max="8" step="0.1" value="3.0">
            </label>
            <label>Command rate Hz
              <input id="quad-command-rate" type="number" min="1" max="60" step="1" value="30">
            </label>
            <label>Vertical speed m/s
              <input id="quad-vertical-speed" type="number" min="0" max="3" step="0.1" value="3.0">
            </label>
            <label>Image gain
              <input id="quad-vertical-gain" type="number" min="0" max="8" step="0.1" value="3.5">
            </label>
          </div>
        </div>
        <div id="plane-fields" class="vehicle-fields" hidden>
          <div class="field-title">Fixed-Wing Tracking Tuning</div>
          <label>Tracking Hz
            <input id="plane-command-rate" type="number" min="1" max="60" step="1" value="30">
          </label>
          <div class="tuning-grid" id="plane-tuning"></div>
          <div class="tuning-actions">
            <button class="primary" type="button" onclick="applyTuning()">APPLY TUNING</button>
            <button type="button" onclick="revertTuning()">REVERT</button>
            <input id="tuning-state" value="Applied" readonly>
          </div>
          <div class="graph-panel">
            <h2>Gain Schedule Preview</h2>
            <canvas id="gain-preview" width="520" height="160"></canvas>
            <p class="muted">Target proximity is based on apparent bounding-box size, not physical range.</p>
          </div>
          <div class="tuning-live" id="tuning-live"></div>
        </div>
        <div class="controls">
          <button onclick="clearSelection()">Clear Selection</button>
        </div>
      </section>
    </div>
    <div class="camera-wrap">
      <section>
        <h2>Camera Target View</h2>
        <label>Stream refresh FPS
          <input id="stream-fps" type="number" min="1" max="60" step="1" value="30">
        </label>
        <div class="mode-row">
          <label>HUD
            <select id="hud-enabled">
              <option value="1" selected>On</option>
              <option value="0">Off</option>
            </select>
          </label>
          <label>HUD Mode
            <select id="hud-mode">
              <option value="operational" selected>Operational</option>
              <option value="diagnostic">Diagnostic</option>
            </select>
          </label>
        </div>
        <div class="camera-stage" id="camera-stage">
          <img id="camera" src="/api/frame.jpg" alt="camera frame" draggable="false">
          <div id="selection-box"></div>
        </div>
      </section>
      <div class="two">
        <section>
          <h2>Steering Log</h2>
          <pre id="steer-log"></pre>
        </section>
        <section>
          <h2>System Log</h2>
          <pre id="system-log"></pre>
        </section>
      </div>
    </div>
  </main>
  <script>
    let currentVehicleMode = 'quad';
    let currentVideoSource = 'rtsp';
    let appliedTuning = {};
    let pendingTuning = {};
    const tuningFields = [
      ['Basic', true, [
        ['plane_airspeed_mps', 'Target Airspeed', 5, 40, 0.1, 'Target FBWA airspeed request used by the throttle governor.'],
        ['vertical_gain', 'Overall Response', 0, 80, 1, 'Technical: vertical_gain. Shared roll/pitch image response.'],
        ['plane_centering_gain', 'Centering Gain', 0, 4, 0.05, 'Technical: plane_centering_gain. Moves the target toward the crosshair.'],
        ['plane_damping_gain', 'Damping', 0, 3, 0.01, 'Technical: plane_damping_gain. Opposes image-error rate.'],
        ['max_plane_roll_deg', 'Max Roll', 1, 35, 1, 'Technical: max_plane_roll_deg. Additional Vulture-X limit.'],
        ['max_plane_pitch_deg', 'Max Pitch', 1, 40, 1, 'Technical: max_plane_pitch_deg. Additional Vulture-X limit.'],
        ['plane_pitch_filter_alpha', 'Command Smoothing', 0.05, 0.8, 0.05, 'Technical: plane_pitch_filter_alpha.'],
      ]],
      ['Tracking Response', true, [
        ['plane_near_centering_gain', 'Near Target Centering', 0, 4, 0.05, 'Technical: plane_near_centering_gain.'],
        ['plane_roll_gain_scale', 'Roll Response', 0, 3, 0.05, 'Technical: plane_roll_gain_scale.'],
        ['plane_pitch_gain_scale', 'Pitch Response', 0, 3, 0.05, 'Technical: plane_pitch_gain_scale.'],
        ['plane_pitch_near_gain_scale', 'Near Target Pitch Response', 0, 5, 0.05, 'Technical: plane_pitch_near_gain_scale.'],
        ['plane_error_deadband', 'Deadband', 0, 0.2, 0.001, 'Technical: plane_error_deadband.'],
        ['plane_lead_s', 'Prediction / Lead', 0, 1, 0.01, 'Technical: plane_lead_s.'],
      ]],
      ['Damping & Smoothing', false, [
        ['plane_near_damping_gain', 'Near Target Damping', 0, 3, 0.01, 'Technical: plane_near_damping_gain.'],
        ['plane_max_pitch_step_deg', 'Max Pitch Change / Frame', 1, 8, 0.5, 'Technical: plane_max_pitch_step_deg.'],
        ['plane_max_roll_step_deg', 'Max Roll Change / Frame', 1, 10, 0.5, 'Technical: plane_max_roll_step_deg.'],
      ]],
      ['Near Target', false, [
        ['plane_near_pitch_down_limit_deg', 'Near Pitch Down Limit', 0, 40, 1, 'Technical: plane_near_pitch_down_limit_deg.'],
        ['plane_far_control_scale', 'Far Control Scale', 0.1, 1, 0.05, 'Technical: plane_far_control_scale.'],
        ['plane_near_control_scale', 'Near Control Scale', 0.1, 1, 0.05, 'Technical: plane_near_control_scale.'],
      ]],
      ['Airspeed & Throttle', false, [
        ['plane_throttle', 'Cruise Throttle', 0, 1, 0.01, 'Technical: plane_throttle.'],
        ['plane_throttle_airspeed_gain', 'Airspeed Correction Gain', 0, 0.2, 0.005, 'Technical: plane_throttle_airspeed_gain.'],
        ['plane_min_throttle', 'Minimum Throttle', 0, 1, 0.01, 'Technical: plane_min_throttle.'],
        ['plane_max_throttle', 'Maximum Throttle', 0, 1, 0.01, 'Technical: plane_max_throttle.'],
        ['plane_near_throttle_reduction', 'Near Target Throttle Reduction', 0, 0.5, 0.01, 'Technical: plane_near_throttle_reduction.'],
      ]],
      ['Camera / Target Geometry', false, [
        ['plane_camera_hfov_deg', 'Camera HFOV', 20, 140, 1, 'Technical: plane_camera_hfov_deg.'],
        ['plane_proximity_far_size', 'Far Target Size', 0.001, 0.5, 0.001, 'Technical: plane_proximity_far_size. Apparent image size only.'],
        ['plane_proximity_near_size', 'Near Target Size', 0.002, 0.8, 0.001, 'Technical: plane_proximity_near_size. Apparent image size only.'],
      ]],
      ['Safety', false, [
        ['plane_loss_hold_s', 'Loss Hold', 0, 5, 0.1, 'Technical: plane_loss_hold_s.'],
        ['min_tracking_alt_m', 'Minimum Tracking Altitude', 0, 200, 1, 'Technical: min_tracking_alt_m. Safety gate only.'],
        ['airspeed_low_persistence_s', 'Low Airspeed Persistence', 0.2, 10, 0.1, 'Technical: airspeed_low_persistence_s.'],
      ]],
    ];
    function allTuningKeys() {
      return tuningFields.flatMap(group => group[2].map(field => field[0]));
    }
    function activeNumber(id, fallback) {
      const element = document.getElementById(id);
      return encodeURIComponent(element ? (element.value || fallback) : fallback);
    }
    function tuningValue(key, fallback) {
      const value = pendingTuning[key] ?? appliedTuning[key] ?? fallback;
      return encodeURIComponent(value);
    }
    function buildTuningPanel() {
      const root = document.getElementById('plane-tuning');
      if (!root || root.childElementCount) return;
      root.innerHTML = tuningFields.map(([group, open, fields]) => `
        <details class="tuning-group" ${open ? 'open' : ''}>
          <summary>${group}</summary>
          ${fields.map(([key, label, min, max, step, help]) => `
            <div class="tuning-field">
              <label for="tune-${key}" title="${key}">${label}</label>
              <input id="tune-${key}" data-tune="${key}" type="range" min="${min}" max="${max}" step="${step}">
              <input id="num-${key}" data-tune-number="${key}" type="number" min="${min}" max="${max}" step="${step}">
              <small>${help}</small>
            </div>
          `).join('')}
        </details>
      `).join('');
      root.querySelectorAll('[data-tune]').forEach((slider) => {
        slider.addEventListener('input', () => setPendingTuning(slider.dataset.tune, slider.value));
      });
      root.querySelectorAll('[data-tune-number]').forEach((input) => {
        input.addEventListener('input', () => setPendingTuning(input.dataset.tuneNumber, input.value));
      });
    }
    function setPendingTuning(key, value) {
      pendingTuning[key] = Number(value);
      syncTuningInputs();
      markTuning('Pending changes');
      drawGainPreview();
    }
    function syncTuningInputs() {
      for (const key of allTuningKeys()) {
        const value = pendingTuning[key] ?? appliedTuning[key];
        if (value === undefined) continue;
        const slider = document.getElementById('tune-' + key);
        const number = document.getElementById('num-' + key);
        if (slider && document.activeElement !== slider) slider.value = value;
        if (number && document.activeElement !== number) number.value = value;
      }
    }
    function markTuning(text) {
      const element = document.getElementById('tuning-state');
      if (element) element.value = text;
    }
    async function loadTuning() {
      buildTuningPanel();
      const response = await fetch('/api/tracking_tuning');
      const payload = await response.json();
      appliedTuning = payload.values || {};
      pendingTuning = {...appliedTuning};
      syncTuningInputs();
      markTuning('Applied');
      drawGainPreview();
    }
    async function applyTuning() {
      const response = await fetch('/api/tracking_tuning', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(pendingTuning)
      });
      const payload = await response.json();
      if (payload.ok) {
        appliedTuning = payload.values || {};
        pendingTuning = {...appliedTuning};
        syncTuningInputs();
        markTuning('Applied r' + payload.revision);
      } else {
        markTuning('Rejected');
      }
      drawGainPreview();
    }
    function revertTuning() {
      pendingTuning = {...appliedTuning};
      syncTuningInputs();
      markTuning('Applied');
      drawGainPreview();
    }
    function drawGainPreview() {
      const canvas = document.getElementById('gain-preview');
      if (!canvas) return;
      const ctx = canvas.getContext('2d');
      const w = canvas.width;
      const h = canvas.height;
      ctx.clearRect(0, 0, w, h);
      ctx.strokeStyle = '#354044';
      ctx.beginPath();
      ctx.moveTo(34, 12); ctx.lineTo(34, h - 24); ctx.lineTo(w - 10, h - 24); ctx.stroke();
      const v = (key, fallback) => Number(pendingTuning[key] ?? fallback);
      const curves = [
        ['#60d394', v('plane_centering_gain', 1.55), v('plane_near_centering_gain', 2.65)],
        ['#eec643', v('plane_damping_gain', 0.14), v('plane_near_damping_gain', 0.30)],
        ['#69b7ff', v('plane_pitch_gain_scale', 1.20), v('plane_pitch_near_gain_scale', 1.60)],
        ['#ff6b6b', v('plane_far_control_scale', 0.55), 1.0],
      ];
      curves.forEach(([color, far, near]) => {
        const maxValue = Math.max(1, far, near);
        ctx.strokeStyle = color;
        ctx.beginPath();
        for (let i = 0; i <= 100; i++) {
          const p = i / 100;
          const yValue = (far + (near - far) * p) / maxValue;
          const x = 34 + p * (w - 44);
          const y = (h - 24) - yValue * (h - 40);
          if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
        }
        ctx.stroke();
      });
    }
    function mavlinkEndpoint() {
      const element = document.getElementById('mavlink-endpoint');
      const fallback = 'udpcl:192.168.144.12:19856';
      return encodeURIComponent(element ? (element.value || fallback) : fallback);
    }
    function videoSource() {
      return currentVideoSource || 'rtsp';
    }
    function rtspUrl() {
      const element = document.getElementById('rtsp-url');
      return encodeURIComponent(element ? (element.value || '') : '');
    }
    async function post(path) {
      const response = await fetch(path, {method: 'POST'});
      const data = await response.json();
      refresh(data);
    }
    async function steer() {
      await startSteering(false);
    }
    async function groundTest() {
      await startSteering(true);
    }
    async function startSteering(surfaceTestMode) {
      const trackingModeInput = document.getElementById('tracking-mode');
      const defaultMode = 'red';
      const mode = encodeURIComponent(
        trackingModeInput ? (trackingModeInput.value || defaultMode) : defaultMode
      );
      const isPlane = currentVehicleMode === 'plane';
      const speed = isPlane
        ? '0'
        : activeNumber('quad-forward-speed', '3.0');
      let rate = isPlane
        ? activeNumber('plane-command-rate', '30')
        : activeNumber('quad-command-rate', '30');
      const verticalSpeed = isPlane
        ? '0'
        : activeNumber('quad-vertical-speed', '3.0');
      const verticalGain = isPlane
        ? tuningValue('vertical_gain', '52')
        : activeNumber('quad-vertical-gain', '3.5');
      let path =
        '/api/start_steering?mavlink=' + mavlinkEndpoint() +
        '&forward_mps=' + speed +
        '&rate_hz=' + rate +
        '&max_down_mps=' + verticalSpeed +
        '&vertical_gain=' + verticalGain +
        '&tracking_mode=' + mode +
        '&surface_test=' + (surfaceTestMode ? '1' : '0');
      if (isPlane) {
        path +=
          '&plane_airspeed_mps=' + tuningValue('plane_airspeed_mps', '20.0') +
          '&plane_centering_gain=' + tuningValue('plane_centering_gain', '1.55') +
          '&plane_near_centering_gain=' + tuningValue('plane_near_centering_gain', '2.65') +
          '&plane_damping_gain=' + tuningValue('plane_damping_gain', '0.14') +
          '&plane_near_damping_gain=' + tuningValue('plane_near_damping_gain', '0.30') +
          '&plane_far_control_scale=' + tuningValue('plane_far_control_scale', '0.72') +
          '&max_plane_roll_deg=' + tuningValue('max_plane_roll_deg', '35') +
          '&max_plane_pitch_deg=' + tuningValue('max_plane_pitch_deg', '40') +
          '&plane_pitch_gain_scale=' + tuningValue('plane_pitch_gain_scale', '1.20') +
          '&plane_pitch_near_gain_scale=' + tuningValue('plane_pitch_near_gain_scale', '1.60') +
          '&plane_pitch_below_center_boost=' + tuningValue('plane_pitch_below_center_boost', '0.25') +
          '&plane_pitch_filter_alpha=' + tuningValue('plane_pitch_filter_alpha', '0.45') +
          '&plane_max_pitch_step_deg=' + tuningValue('plane_max_pitch_step_deg', '2.0') +
          '&plane_max_roll_step_deg=' + tuningValue('plane_max_roll_step_deg', '6.0') +
          '&plane_loss_hold_s=' + tuningValue('plane_loss_hold_s', '1.5');
      }
      await post(path);
    }
    async function connectMavlink() {
      await post('/api/connect_mavlink?mavlink=' + mavlinkEndpoint());
    }
    async function connectVideo() {
      await post('/api/start_bridge?video_source=' + videoSource() + '&rtsp_url=' + rtspUrl());
    }
    function badge(value, good) {
      const cls = good ? 'ok' : 'bad';
      return `<span class="pill ${cls}">${value}</span>`;
    }
    function row(name, value, good) {
      return `<div class="row"><span>${name}</span>${badge(value, good)}</div>`;
    }
    function refresh(data) {
      if (!data) return;
      const target = data.target.detected
        ? `detected ${data.target.center_x.toFixed(3)},${data.target.center_y.toFixed(3)}`
        : 'not detected';
      document.getElementById('status').innerHTML = [
        row(
          'Aircraft',
          data.vehicle.label,
          data.vehicle.mode === 'quad' || data.vehicle.mode === 'plane'
        ),
        row('Camera Bridge', data.processes.bridge ? 'running' : 'stopped', data.processes.bridge),
        row(
          'Video Input',
          data.video.input_label || data.video.source.toUpperCase(),
          data.camera.live
        ),
        ...(data.runtime.simulator ? [
          row('Gazebo', data.processes.gazebo ? 'running' : 'stopped', data.processes.gazebo),
          row('SITL', data.processes.sitl ? 'running' : 'stopped', data.processes.sitl),
          row(
            'Target Motion',
            data.processes.target_motion ? 'running' : 'idle',
            true
          ),
          row('Takeoff', data.processes.takeoff ? 'running' : 'idle', true),
        ] : []),
        row('Live Frames', data.camera.live ? 'live' : 'stale', data.camera.live),
        row('Target', target, data.target.detected),
        row('Steering', data.processes.steering ? 'running' : 'idle', !data.processes.steering),
      ].join('');
      updateRuntimePerformance(data.demand || {});
      currentVehicleMode = data.vehicle.mode;
      currentVideoSource = data.video.source || currentVideoSource;
      document.getElementById('quad-fields').hidden = currentVehicleMode !== 'quad';
      document.getElementById('plane-fields').hidden = currentVehicleMode !== 'plane';
      document.getElementById('ground-test-btn').hidden = currentVehicleMode !== 'plane';
      document.getElementById('sim-actions').hidden = !data.runtime.simulator;
      document.getElementById('takeoff-btn').hidden =
        currentVehicleMode !== 'plane' || !data.runtime.simulator;
      updateTuningLive(data.demand || {});
      const endpointInput = document.getElementById('mavlink-endpoint');
      if (endpointInput && document.activeElement !== endpointInput) {
        endpointInput.value = data.mavlink.control_endpoint || endpointInput.value;
      }
      const rtspUrlInput = document.getElementById('rtsp-url');
      if (rtspUrlInput && document.activeElement !== rtspUrlInput) {
        rtspUrlInput.value = data.video.rtsp_url || rtspUrlInput.value;
      }
      const rtspLabel = document.getElementById('rtsp-url-label');
      if (rtspLabel && rtspUrlInput) {
        const isRtsp = data.video.source === 'rtsp';
        rtspLabel.hidden = !isRtsp;
        rtspUrlInput.disabled = !isRtsp;
      }
      document.getElementById('steer-log').textContent = data.logs.steering;
      const selectionText = data.selection.enabled
        ? `${data.selection.width.toFixed(3)} x ${data.selection.height.toFixed(3)}`
        : 'none';
      document.getElementById('selection-status').value = selectionText;
      drawStoredSelection(data.selection);
      const owners = data.camera.udp_5600_owners.join('\n');
      document.getElementById('system-log').textContent =
        data.logs.system +
        (data.logs.takeoff ? '\n\nTakeoff:\n' + data.logs.takeoff : '') +
        (owners ? '\n\nUDP 5600:\n' + owners : '');
    }
    function updateTuningLive(demand) {
      const element = document.getElementById('tuning-live');
      if (!element) return;
      const item = (label, value) => `<span>${label}</span><strong>${value ?? '-'}</strong>`;
      const warn = demand.failsafe ? `control released: ${demand.failsafe_reason}` :
        (!demand.detected ? 'target lost or searching' : 'tracking');
      element.innerHTML = [
        item('Target error X', Number(demand.guided_error_x ?? 0).toFixed(3)),
        item('Target error Y', Number(demand.guided_error_y ?? 0).toFixed(3)),
        item('Proximity', Number(demand.target_proximity ?? 0).toFixed(2)),
        item('Roll command', Number(demand.roll_deg ?? 0).toFixed(1)),
        item('Pitch command', Number(demand.pitch_deg ?? 0).toFixed(1)),
        item('Throttle', Number(demand.throttle ?? 0).toFixed(2)),
        item('Airspeed', demand.airspeed_mps == null ? '-' : Number(demand.airspeed_mps).toFixed(1)),
        item('Frame age', demand.frame_age_ms == null ? '-' : Number(demand.frame_age_ms).toFixed(0) + ' ms'),
        item('Revision', demand.tracking_tuning_revision ?? '-'),
        item('State', warn),
      ].join('');
    }
    function metricValue(metrics, key, suffix, decimals = 1) {
      const value = metrics ? metrics[key] : null;
      return Number.isFinite(Number(value)) ? Number(value).toFixed(decimals) + suffix : '-';
    }
    function updateRuntimePerformance(demand) {
      const element = document.getElementById('runtime-performance');
      if (!element) return;
      const metrics = demand.runtime_metrics || {};
      element.innerHTML = [
        row('Camera FPS', metricValue(metrics, 'capture_hz', ' Hz'), true),
        row('Vision FPS', metricValue(metrics, 'vision_hz', ' Hz'), true),
        row('Control Hz', metricValue(metrics, 'control_hz', ' Hz'), true),
        row('MAVLink TX', metricValue(metrics, 'mavlink_tx_hz', ' Hz'), true),
        row('Vision P95', metricValue(metrics, 'vision_ms_p95', ' ms'), true),
        row('Frame Age', demand.frame_age_ms == null ? '-' : Number(demand.frame_age_ms).toFixed(0) + ' ms', true),
        row('Result Age', demand.tracking_result_age_ms == null ? '-' : Number(demand.tracking_result_age_ms).toFixed(0) + ' ms', true),
        row('Jitter P95', metricValue(metrics, 'control_jitter_ms_p95', ' ms'), true),
        row('Dropped Frames', String(metrics.dropped_frames ?? 0), Number(metrics.dropped_frames ?? 0) === 0),
        row('Deadline Misses', String(metrics.deadline_misses ?? 0), Number(metrics.deadline_misses ?? 0) === 0),
        row('Tracker Failures', String(metrics.tracker_failures ?? 0), Number(metrics.tracker_failures ?? 0) === 0),
      ].join('');
    }
    function refreshFrame() {
      const mode = encodeURIComponent(document.getElementById('tracking-mode').value || 'red');
      const hudMode = encodeURIComponent(document.getElementById('hud-mode').value || 'operational');
      const hud = encodeURIComponent(document.getElementById('hud-enabled').value || '1');
      document.getElementById('camera').src =
        '/api/frame.jpg?mode=' + mode + '&hud_mode=' + hudMode + '&hud=' + hud + '&t=' + Date.now();
    }
    async function saveSelection(selection) {
      const response = await fetch('/api/selection', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(selection)
      });
      refresh(await response.json());
      document.getElementById('tracking-mode').value = 'custom';
    }
    async function clearSelection() {
      const response = await fetch('/api/selection/clear', {method: 'POST'});
      refresh(await response.json());
      document.getElementById('tracking-mode').value = 'red';
    }
    async function selectHeadAt(point) {
      const content = imageContentRect();
      const x = clampUnit((point.x - content.left) / content.width);
      const y = clampUnit((point.y - content.top) / content.height);
      const response = await fetch('/api/selection/head?x=' + x + '&y=' + y, {method: 'POST'});
      refresh(await response.json());
      document.getElementById('tracking-mode').value = 'head';
    }
    function imageContentRect() {
      const img = document.getElementById('camera');
      const rect = img.getBoundingClientRect();
      const naturalRatio = img.naturalWidth / img.naturalHeight;
      const renderedRatio = rect.width / rect.height;
      let width = rect.width;
      let height = rect.height;
      let left = rect.left;
      let top = rect.top;
      if (renderedRatio > naturalRatio) {
        width = rect.height * naturalRatio;
        left += (rect.width - width) / 2;
      } else if (renderedRatio < naturalRatio) {
        height = rect.width / naturalRatio;
        top += (rect.height - height) / 2;
      }
      return {left, top, width, height};
    }
    function clampUnit(value) {
      return Math.max(0, Math.min(1, value));
    }
    function drawBoxFromPixels(start, end) {
      const content = imageContentRect();
      const x1 = Math.max(content.left, Math.min(content.left + content.width, start.x));
      const y1 = Math.max(content.top, Math.min(content.top + content.height, start.y));
      const x2 = Math.max(content.left, Math.min(content.left + content.width, end.x));
      const y2 = Math.max(content.top, Math.min(content.top + content.height, end.y));
      const box = document.getElementById('selection-box');
      const stage = document.getElementById('camera-stage').getBoundingClientRect();
      box.style.display = 'block';
      box.style.left = (Math.min(x1, x2) - stage.left) + 'px';
      box.style.top = (Math.min(y1, y2) - stage.top) + 'px';
      box.style.width = Math.abs(x2 - x1) + 'px';
      box.style.height = Math.abs(y2 - y1) + 'px';
    }
    function drawStoredSelection(selection) {
      const box = document.getElementById('selection-box');
      if (!selection.enabled) {
        box.style.display = 'none';
        return;
      }
      const content = imageContentRect();
      const stage = document.getElementById('camera-stage').getBoundingClientRect();
      box.style.display = 'block';
      box.style.left = (content.left + selection.x * content.width - stage.left) + 'px';
      box.style.top = (content.top + selection.y * content.height - stage.top) + 'px';
      box.style.width = (selection.width * content.width) + 'px';
      box.style.height = (selection.height * content.height) + 'px';
    }
    function installSelectionDrag() {
      const stage = document.getElementById('camera-stage');
      let start = null;
      let dragging = false;
      stage.addEventListener('pointerdown', (event) => {
        event.preventDefault();
        start = {x: event.clientX, y: event.clientY};
        dragging = true;
        stage.setPointerCapture(event.pointerId);
        drawBoxFromPixels(start, start);
      });
      stage.addEventListener('pointermove', (event) => {
        if (!dragging || !start) return;
        drawBoxFromPixels(start, {x: event.clientX, y: event.clientY});
      });
      stage.addEventListener('pointerup', async (event) => {
        if (!dragging || !start) return;
        dragging = false;
        const distance = Math.hypot(event.clientX - start.x, event.clientY - start.y);
        const mode = document.getElementById('tracking-mode').value || 'red';
        if (mode === 'head' && distance < 8) {
          await selectHeadAt({x: event.clientX, y: event.clientY});
          return;
        }
        const content = imageContentRect();
        const x1 = clampUnit((start.x - content.left) / content.width);
        const y1 = clampUnit((start.y - content.top) / content.height);
        const x2 = clampUnit((event.clientX - content.left) / content.width);
        const y2 = clampUnit((event.clientY - content.top) / content.height);
        const selection = {
          x: Math.min(x1, x2),
          y: Math.min(y1, y2),
          width: Math.abs(x2 - x1),
          height: Math.abs(y2 - y1)
        };
        if (selection.width < 0.02 || selection.height < 0.02) {
          await clearSelection();
          return;
        }
        await saveSelection(selection);
      });
    }
    function streamPeriodMs() {
      const fps = Math.max(
        1,
        Math.min(60, Number(document.getElementById('stream-fps').value || '30'))
      );
      return Math.round(1000 / fps);
    }
    let frameTimer = window.setInterval(refreshFrame, streamPeriodMs());
    document.getElementById('stream-fps').addEventListener('change', () => {
      window.clearInterval(frameTimer);
      frameTimer = window.setInterval(refreshFrame, streamPeriodMs());
    });
    async function poll() {
      try {
        const mode = encodeURIComponent(document.getElementById('tracking-mode').value || 'red');
        const hudMode = encodeURIComponent(document.getElementById('hud-mode').value || 'operational');
        const hud = encodeURIComponent(document.getElementById('hud-enabled').value || '1');
        const response = await fetch('/api/status?mode=' + mode + '&hud_mode=' + hudMode + '&hud=' + hud);
        refresh(await response.json());
      } catch (error) {
        document.getElementById('system-log').textContent = String(error);
      }
    }
    setInterval(poll, 1000);
    installSelectionDrag();
    loadTuning().catch((error) => markTuning('Rejected'));
    poll();
    refreshFrame();
  </script>
</body>
</html>
"""


class ManagedProcess:
    def __init__(
        self,
        name: str,
        command: list[str],
        log_path: Path,
        env: dict[str, str] | None = None,
        terminal: bool = False,
    ):
        self.name = name
        self.command = command
        self.log_path = log_path
        self.env = env
        self.terminal = terminal
        self.process: subprocess.Popen[str] | None = None

    def running(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def start(self) -> str:
        if self.running():
            return f"{self.name} already running"
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        started_at = time.strftime("%Y-%m-%d %H:%M:%S")
        with self.log_path.open("a", encoding="utf-8") as log_file:
            log_file.write(f"\n[{started_at}] starting {' '.join(self.command)}\n")
            log_file.flush()
        process_env = os.environ.copy()
        if self.env:
            process_env.update(self.env)
        terminal_command = terminal_launcher(self.name, self.command, self.log_path)
        if self.terminal and terminal_command is not None:
            self.process = subprocess.Popen(
                terminal_command,
                cwd=REPO_ROOT,
                env=process_env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                text=True,
                start_new_session=True,
            )
        else:
            with self.log_path.open("a", encoding="utf-8") as log_file:
                self.process = subprocess.Popen(
                    self.command,
                    cwd=REPO_ROOT,
                    env=process_env,
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                    text=True,
                    start_new_session=True,
                )
        return f"{self.name} started"

    def stop(self) -> str:
        if not self.running() or self.process is None:
            return f"{self.name} not running"
        try:
            os.killpg(self.process.pid, signal.SIGTERM)
        except ProcessLookupError:
            return f"{self.name} stopped"
        try:
            self.process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            with contextlib.suppress(ProcessLookupError):
                os.killpg(self.process.pid, signal.SIGKILL)
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                return f"{self.name} did not exit before timeout"
        return f"{self.name} stopped"


class UiSelectionTracker:
    def __init__(self, tracker_name: str = "TEMPLATE") -> None:
        self._tracker_name = tracker_name
        self._tracker: Any | None = None
        self._selection_mtime_ns: int | None = None
        self._last_frame_key: tuple[str, int] | None = None
        self._last_bbox: tuple[int, int, int, int] | None = None
        self._last_frame_shape: tuple[int, int] | None = None
        self._misses = 0
        self._max_held_misses = 12

    def reset(self) -> None:
        self._tracker = None
        self._selection_mtime_ns = None
        self._last_frame_key = None
        self._last_bbox = None
        self._last_frame_shape = None
        self._misses = 0

    def payload(self) -> dict[str, float | bool | str] | None:
        if self._last_bbox is None or self._last_frame_shape is None:
            return None
        selection = load_selection()
        mode = str(selection.get("mode", "custom")) if selection is not None else "custom"
        frame_height, frame_width = self._last_frame_shape
        x, y, width, height = self._last_bbox
        return {
            "enabled": True,
            "mode": mode,
            "tracking": "locked",
            "x": x / frame_width,
            "y": y / frame_height,
            "width": width / frame_width,
            "height": height / frame_height,
        }

    def bbox(self, frame: Any, image_path: Path) -> tuple[int, int, int, int] | None:
        selection = load_selection()
        if selection is None:
            self.reset()
            return None

        try:
            selection_mtime_ns = SELECTION_PATH.stat().st_mtime_ns
            frame_key = (str(image_path), image_path.stat().st_mtime_ns)
        except FileNotFoundError:
            return None

        if self._last_frame_key == frame_key:
            return self._last_bbox

        if self._tracker is None or self._selection_mtime_ns != selection_mtime_ns:
            bbox = refine_bbox_to_salient_region(
                frame,
                normalized_selection_to_bbox(selection, frame),
            )
            tracker = create_cv_tracker(self._tracker_name)
            if tracker.init(frame, bbox) is False:
                self.reset()
                return None
            self._tracker = tracker
            self._selection_mtime_ns = selection_mtime_ns
            self._last_frame_key = frame_key
            self._last_bbox = bbox
            self._last_frame_shape = frame.shape[:2]
            self._misses = 0
            return bbox

        detected, raw_bbox = self._tracker.update(frame)
        self._last_frame_key = frame_key
        if not detected:
            self._misses += 1
            if self._last_bbox is not None and self._misses <= self._max_held_misses:
                self._last_frame_shape = frame.shape[:2]
                return self._last_bbox
            self._tracker = None
            self._last_frame_shape = frame.shape[:2]
            return None
        frame_height, frame_width = frame.shape[:2]
        self._misses = 0
        self._last_bbox = clamp_bbox(
            tuple(round(value) for value in raw_bbox),
            frame_width,
            frame_height,
        )
        self._last_frame_shape = frame.shape[:2]
        return self._last_bbox


class RuntimeSteeringSession:
    """In-process fixed-wing runtime adapter used by WebUI START TRACKING."""

    def __init__(
        self,
        *,
        mavlink_endpoint: str,
        video_source: str,
        rtsp_url: str,
        camera_dir: Path | None,
        tracking_mode: str,
        rate_hz: float,
        surface_test: bool,
        tuning_values: dict[str, object],
        simulator_mode: bool,
    ) -> None:
        self.mavlink_endpoint = mavlink_endpoint
        self.video_source = video_source
        self.rtsp_url = rtsp_url
        self.camera_dir = camera_dir
        self.tracking_mode = "red" if tracking_mode == "orange" else tracking_mode
        self.rate_hz = rate_hz
        self.surface_test = surface_test
        self.simulator_mode = simulator_mode
        self.connection: mavutil.mavfile | None = None
        self.runtime: TrackingRuntime | None = None
        self.running = False
        self.abort_reason: str | None = None
        self.control_authority_active = False
        self.plane_tracking_started = False
        self.current_mode: str | None = None
        self.last_detection_ns: int | None = None
        self.previous_error_x: float | None = None
        self.previous_error_y: float | None = None
        self.previous_error_time_ns: int | None = None
        self.previous_roll_command_deg: float | None = 0.0
        self.previous_pitch_command_deg: float | None = 0.0
        self.low_airspeed_since_ns: int | None = None
        self.last_command_ns: int | None = None
        self.tracking_sequence = 0
        self.tuning = stt.clamp_plane_tuning(
            stt.PlaneTrackingTuning(
                revision=int(tuning_values.get("revision", 0)),
                vertical_gain=float(tuning_values["vertical_gain"]),
                plane_centering_gain=float(tuning_values["plane_centering_gain"]),
                plane_near_centering_gain=float(tuning_values["plane_near_centering_gain"]),
                plane_roll_gain_scale=float(tuning_values["plane_roll_gain_scale"]),
                plane_pitch_gain_scale=float(tuning_values["plane_pitch_gain_scale"]),
                plane_pitch_near_gain_scale=float(tuning_values["plane_pitch_near_gain_scale"]),
                plane_error_deadband=float(tuning_values.get("plane_error_deadband", 0.015)),
                plane_lead_s=float(tuning_values.get("plane_lead_s", 0.0)),
                plane_damping_gain=float(tuning_values["plane_damping_gain"]),
                plane_near_damping_gain=float(tuning_values["plane_near_damping_gain"]),
                plane_pitch_filter_alpha=float(tuning_values["plane_pitch_filter_alpha"]),
                plane_max_pitch_step_deg=float(tuning_values["plane_max_pitch_step_deg"]),
                plane_max_roll_step_deg=float(tuning_values["plane_max_roll_step_deg"]),
                max_plane_roll_deg=float(tuning_values["max_plane_roll_deg"]),
                max_plane_pitch_deg=float(tuning_values["max_plane_pitch_deg"]),
                plane_near_pitch_down_limit_deg=float(
                    tuning_values.get("plane_near_pitch_down_limit_deg", 40.0)
                ),
                plane_far_control_scale=float(tuning_values["plane_far_control_scale"]),
                plane_near_control_scale=float(tuning_values.get("plane_near_control_scale", 1.0)),
                plane_camera_hfov_deg=float(tuning_values.get("plane_camera_hfov_deg", 70.0)),
                plane_proximity_far_size=float(
                    tuning_values.get("plane_proximity_far_size", 0.025)
                ),
                plane_proximity_near_size=float(
                    tuning_values.get("plane_proximity_near_size", 0.16)
                ),
                plane_airspeed_mps=float(tuning_values["plane_airspeed_mps"]),
                plane_throttle=float(tuning_values["plane_throttle"]),
                plane_throttle_airspeed_gain=float(
                    tuning_values.get("plane_throttle_airspeed_gain", 0.04)
                ),
                plane_min_throttle=float(tuning_values["plane_min_throttle"]),
                plane_max_throttle=float(tuning_values["plane_max_throttle"]),
                plane_near_throttle_reduction=float(
                    tuning_values.get("plane_near_throttle_reduction", 0.0)
                ),
                plane_pitch_below_center_boost=float(
                    tuning_values["plane_pitch_below_center_boost"]
                ),
                plane_loss_hold_s=float(tuning_values["plane_loss_hold_s"]),
                min_tracking_alt_m=float(tuning_values.get("min_tracking_alt_m", 15.0)),
                airspeed_low_persistence_s=float(
                    tuning_values.get("airspeed_low_persistence_s", 2.0)
                ),
            )
        )
        self.response_model = stt.plane_response_model_from_params(self.tuning, {})
        self.tracker: object | None = None
        self.tracker_engine_name = "unknown"

    def start(self) -> str:
        if self.running:
            return "runtime steering already running"
        self.connection = open_mavlink_connection(
            self.mavlink_endpoint,
            source_system=255,
            source_component=194,
            autoreconnect=False,
        )
        args = self._verification_args()
        target_system, target_component, vehicle = stt.verify_connection(self.connection, args)
        if vehicle != "plane":
            self.close()
            return "runtime steering blocked reason=plane_required"
        param_values = stt.load_plane_param_cache(PLANE_PARAM_CACHE_PATH, self.mavlink_endpoint)
        if param_values is None:
            param_values = stt.read_plane_parameters(
                self.connection,
                target_system,
                target_component,
                timeout_per_param_s=0.12,
            )
            stt.save_plane_param_cache(PLANE_PARAM_CACHE_PATH, self.mavlink_endpoint, param_values)
        block_reason = stt.validate_rc_override_acceptance(param_values, 255)
        if block_reason is not None:
            self.close()
            return f"runtime steering blocked reason={block_reason}"
        self.response_model = stt.plane_response_model_from_params(self.tuning, param_values)
        self.tracker = self._create_tracker()
        source = self._create_frame_source()
        receiver = MavlinkTelemetryReceiver(self.connection)
        self.runtime = TrackingRuntime(
            source,
            self._process_frame,
            receiver.receive,
            self._control_step,
            control_rate_hz=30.0,
        )
        self.runtime.start()
        self.running = True
        return "runtime steering started"

    def stop(self, *, request_auto: bool) -> str:
        self._release_override("normal_stop")
        if self.runtime is not None:
            self.runtime.stop()
        if self.connection is not None and request_auto:
            # Existing WebUI semantics request AUTO on normal stop for simulator plane.
            auto_mode = self.connection.mode_mapping().get("AUTO")
            if auto_mode is not None:
                with contextlib.suppress(Exception):
                    self.connection.set_mode(auto_mode)
        self.close()
        return "runtime steering stopped"

    def close(self) -> None:
        if self.connection is not None:
            with contextlib.suppress(Exception):
                self.connection.close()
        self.connection = None
        self.runtime = None
        self.running = False

    def snapshot(self) -> RuntimeSnapshot | None:
        return self.runtime.snapshot() if self.runtime is not None else None

    def _verification_args(self) -> argparse.Namespace:
        return argparse.Namespace(
            timeout_s=20.0,
            vehicle="plane",
            surface_test=self.surface_test,
        )

    def _create_frame_source(self) -> object:
        backend = os.environ.get("VULTURE_X_RUNTIME_CAPTURE_BACKEND", "appsink")
        if backend == "file":
            if self.camera_dir is None:
                raise RuntimeError("runtime_file_capture_missing_camera_dir")
            return FileFrameSource(self.camera_dir)
        try:
            if self.video_source == "rtsp":
                return GstAppSinkFrameSource(
                    appsink_pipeline_from_rtsp(
                        self.rtsp_url,
                        latency_ms=int(DEFAULT_RTSP_LATENCY_MS),
                        protocols=DEFAULT_RTSP_PROTOCOLS,
                        max_rate=int(DEFAULT_RTSP_MAX_RATE),
                    )
                )
            return GstAppSinkFrameSource(appsink_pipeline_from_udp_h264())
        except Exception:
            if self.camera_dir is None:
                raise
            return FileFrameSource(self.camera_dir, source_name="file-fallback")

    def _create_tracker(self) -> object:
        tracker_name = (
            f"NANO:{DEFAULT_NANO_BACKBONE}:{DEFAULT_NANO_NECKHEAD}"
            if self.tracking_mode in {"custom", "head", "person"}
            else "TEMPLATE"
        )
        self.tracker_engine_name = "NanoTrack" if tracker_name.startswith("NANO:") else "TEMPLATE"
        if self.tracking_mode in {"custom", "head", "person"}:
            return stt.CustomSelectionTracker(tracker_name, SELECTION_PATH)
        detector = detect_banner_target if self.tracking_mode == "banner" else detect_colored_target
        return stt.DetectorBackedTracker("TEMPLATE", 25.0, detector)

    def _process_frame(self, frame: VideoFrame) -> TrackingSnapshot:
        self.tracking_sequence += 1
        bbox = None if self.tracker is None else self.tracker.bbox(frame.image)
        confidence = stt.tracker_confidence(self.tracker)
        if confidence is None:
            confidence = stt.tracker_last_confidence(
                self.tracker if self.tracking_mode in {"custom", "head", "person"} else None,
                self.tracker if self.tracking_mode not in {"custom", "head", "person"} else None,
            )
        if bbox is None:
            return TrackingSnapshot(
                sequence=self.tracking_sequence,
                frame_sequence=frame.sequence,
                timestamp_ns=time.monotonic_ns(),
                detected=False,
                confidence=confidence,
                mode=self.tracking_mode,
                vision_state="LOST" if self.plane_tracking_started else "ACQUIRE",
            )
        x, y, width, height = bbox
        frame_height, frame_width = frame.image.shape[:2]
        center_x = (x + width / 2.0) / frame_width
        center_y = (y + height / 2.0) / frame_height
        error_x = center_x - 0.5
        error_y = center_y - 0.5
        aspect_ratio = frame_width / frame_height
        vertical_fov_deg = math.degrees(
            2.0
            * math.atan(
                math.tan(math.radians(self.tuning.plane_camera_hfov_deg) * 0.5) / aspect_ratio
            )
        )
        guided_error_x = stt.apply_deadband(
            stt.perspective_correct_error(error_x, self.tuning.plane_camera_hfov_deg),
            self.tuning.plane_error_deadband,
        )
        guided_error_y = stt.apply_deadband(
            stt.perspective_correct_error(error_y, vertical_fov_deg),
            self.tuning.plane_error_deadband,
        )
        proximity = stt.target_proximity_from_bbox(
            bbox,
            frame_width,
            frame_height,
            far_size=self.tuning.plane_proximity_far_size,
            near_size=self.tuning.plane_proximity_near_size,
        )
        return TrackingSnapshot(
            sequence=self.tracking_sequence,
            frame_sequence=frame.sequence,
            timestamp_ns=time.monotonic_ns(),
            detected=True,
            bbox=bbox,
            confidence=confidence,
            center_x=center_x,
            center_y=center_y,
            horizontal_error=guided_error_x,
            vertical_error=guided_error_y,
            mode=self.tracking_mode,
            vision_state="TRACK",
            details={"target_proximity": proximity},
        )

    def _control_step(self, inputs: ControlInputs) -> None:
        if self.abort_reason is not None or self.connection is None:
            return
        vehicle = inputs.vehicle
        tracking = inputs.tracking
        now_ns = inputs.now_ns
        if self.plane_tracking_started:
            if inputs.heartbeat_age_ms is None or inputs.heartbeat_age_ms > DEFAULT_MAVLINK_TIMEOUT_MS:
                self._abort("mavlink_heartbeat_timeout")
                return
            if vehicle is not None:
                if vehicle.armed is False and not self.surface_test:
                    self._abort("vehicle_disarmed")
                    return
                if vehicle.mode is not None and vehicle.mode != "FBWA":
                    self._abort("pilot_mode_change")
                    return
        if inputs.frame_age_ms is not None and inputs.frame_age_ms > DEFAULT_MAX_FRAME_AGE_MS:
            self._abort("stale_video")
            return
        if tracking is None or not tracking.detected:
            self._handle_missing_tracking(inputs)
            return
        self.last_detection_ns = now_ns
        if not self._ensure_tracking_started(inputs):
            return
        self._send_tracking_command(inputs)

    def _ensure_tracking_started(self, inputs: ControlInputs) -> bool:
        if self.plane_tracking_started:
            return True
        vehicle = inputs.vehicle
        if vehicle is None:
            return False
        if not self.surface_test and vehicle.armed is not True:
            self._abort("vehicle_disarmed")
            return False
        if not self.surface_test and vehicle.relative_altitude_m is None:
            self._abort("altitude_unknown")
            return False
        if (
            not self.surface_test
            and self.response_model.min_airspeed_mps is not None
            and vehicle.airspeed_mps is None
        ):
            self._abort("airspeed_unknown")
            return False
        if self.connection is None or not stt.request_plane_fbwa(self.connection, vehicle.mode):
            self._abort("fbwa_transition_timeout")
            return False
        self.plane_tracking_started = True
        self.current_mode = "FBWA"
        return True

    def _handle_missing_tracking(self, inputs: ControlInputs) -> None:
        if not self.plane_tracking_started:
            self._write_demand(False, inputs)
            return
        age_s = (
            float("inf")
            if self.last_detection_ns is None
            else (inputs.now_ns - self.last_detection_ns) / 1_000_000_000.0
        )
        if age_s > self.tuning.plane_loss_hold_s:
            self._abort("target_lost")
            return
        self._send_hold_command(inputs)

    def _send_tracking_command(self, inputs: ControlInputs) -> None:
        tracking = inputs.tracking
        vehicle = inputs.vehicle
        if tracking is None or tracking.horizontal_error is None or tracking.vertical_error is None:
            return
        if vehicle is None:
            return
        if not self.surface_test:
            if vehicle.relative_altitude_m is None:
                self._abort("altitude_unknown")
                return
            if vehicle.relative_altitude_m < self.tuning.min_tracking_alt_m:
                self._abort("below_tracking_altitude")
                return
            min_airspeed = self.response_model.min_airspeed_mps
            if min_airspeed is not None and vehicle.airspeed_mps is not None:
                if vehicle.airspeed_mps < min_airspeed:
                    if self.low_airspeed_since_ns is None:
                        self.low_airspeed_since_ns = inputs.now_ns
                    elif (
                        inputs.now_ns - self.low_airspeed_since_ns
                        >= self.tuning.airspeed_low_persistence_s * 1_000_000_000
                    ):
                        self._abort("low_airspeed")
                        return
                else:
                    self.low_airspeed_since_ns = None
        proximity = float(tracking.details.get("target_proximity", 0.0))
        guided_error_x = tracking.horizontal_error
        guided_error_y = tracking.vertical_error
        proportional_error_x = guided_error_x
        proportional_error_y = guided_error_y
        if self.previous_error_time_ns is not None and self.previous_error_x is not None and self.previous_error_y is not None:
            dt_s = max(0.001, (inputs.now_ns - self.previous_error_time_ns) / 1_000_000_000.0)
            damping_gain = stt.scheduled_gain(
                self.tuning.plane_damping_gain,
                self.tuning.plane_near_damping_gain,
                proximity,
            )
            guided_error_x = stt.damped_axis_error(
                proportional_error_x,
                self.previous_error_x,
                dt_s,
                damping_gain,
            )
            guided_error_y = stt.damped_axis_error(
                proportional_error_y,
                self.previous_error_y,
                dt_s,
                damping_gain,
            )
        self.previous_error_x = proportional_error_x
        self.previous_error_y = proportional_error_y
        self.previous_error_time_ns = inputs.now_ns
        control_scale = stt.far_target_control_scale(
            proximity,
            self.tuning.plane_far_control_scale,
        ) * stt.near_target_control_scale(proximity, self.tuning.plane_near_control_scale)
        centering_gain = stt.scheduled_gain(
            self.tuning.plane_centering_gain,
            self.tuning.plane_near_centering_gain,
            proximity,
        )
        roll_deg = stt.clamp(
            guided_error_x
            * self.tuning.vertical_gain
            * centering_gain
            * self.tuning.plane_roll_gain_scale
            * control_scale,
            -self.response_model.max_roll_deg,
            self.response_model.max_roll_deg,
        )
        roll_deg = stt.damp_pitch_command(
            self.previous_roll_command_deg,
            roll_deg,
            self.response_model.pitch_filter_alpha,
            self.tuning.plane_max_roll_step_deg,
        )
        pitch_deg = stt.plane_pitch_command(
            guided_error_y,
            self.tuning.vertical_gain * centering_gain * control_scale,
            self.tuning.plane_pitch_gain_scale,
            self.tuning.plane_pitch_near_gain_scale,
            proximity,
            self.tuning.plane_pitch_below_center_boost,
            max(self.response_model.max_pitch_up_deg, self.response_model.max_pitch_down_deg),
            self.tuning.plane_near_pitch_down_limit_deg,
        )
        pitch_deg = stt.clamp_plane_pitch(
            pitch_deg,
            self.response_model.max_pitch_up_deg,
            self.response_model.max_pitch_down_deg,
        )
        pitch_deg = stt.damp_pitch_command(
            self.previous_pitch_command_deg,
            pitch_deg,
            self.response_model.pitch_filter_alpha,
            self.response_model.max_pitch_step_deg,
        )
        throttle = (
            DEFAULT_GROUND_TEST_THROTTLE
            if self.surface_test
            else stt.plane_throttle_for_airspeed(
                vehicle.airspeed_mps,
                self.response_model.target_airspeed_mps,
                self.response_model.cruise_throttle,
                self.response_model.min_throttle,
                self.response_model.max_throttle,
                self.tuning.plane_throttle_airspeed_gain,
                pitch_deg,
                max(self.response_model.max_pitch_up_deg, self.response_model.max_pitch_down_deg),
                proximity,
                self.tuning.plane_near_throttle_reduction,
            )
        )
        self._send_rc(roll_deg, pitch_deg, throttle, inputs, guided_error_x, guided_error_y)

    def _send_hold_command(self, inputs: ControlInputs) -> None:
        roll_deg = self.previous_roll_command_deg or 0.0
        pitch_deg = self.previous_pitch_command_deg or 0.0
        vehicle = inputs.vehicle
        throttle = (
            DEFAULT_GROUND_TEST_THROTTLE
            if self.surface_test
            else stt.plane_throttle_for_airspeed(
                None if vehicle is None else vehicle.airspeed_mps,
                self.response_model.target_airspeed_mps,
                self.response_model.cruise_throttle,
                self.response_model.min_throttle,
                self.response_model.max_throttle,
                self.tuning.plane_throttle_airspeed_gain,
                pitch_deg,
                max(self.response_model.max_pitch_up_deg, self.response_model.max_pitch_down_deg),
            )
        )
        self._send_rc(roll_deg, pitch_deg, throttle, inputs, 0.0, 0.0, detected=False)

    def _send_rc(
        self,
        roll_deg: float,
        pitch_deg: float,
        throttle: float,
        inputs: ControlInputs,
        guided_error_x: float,
        guided_error_y: float,
        *,
        detected: bool = True,
    ) -> None:
        if self.connection is None:
            return
        tx_start_ns = time.monotonic_ns()
        stt.send_plane_rc_attitude(
            self.connection,
            roll_deg,
            pitch_deg,
            self.response_model.max_roll_deg,
            max(self.response_model.max_pitch_up_deg, self.response_model.max_pitch_down_deg),
            throttle,
            self.response_model.pitch_rc_reversed,
            self.response_model.roll_channel,
            self.response_model.pitch_channel,
            self.response_model.throttle_channel,
            self.response_model.yaw_channel,
            self.response_model.roll_rc_reversed,
            self.response_model.rc_calibration,
        )
        if self.runtime is not None:
            self.runtime.metrics.observe("mavlink_tx_ms", (time.monotonic_ns() - tx_start_ns) / 1_000_000.0)
            self.runtime.metrics.mark_rate("mavlink_tx", time.monotonic_ns())
        self.control_authority_active = True
        self.previous_roll_command_deg = roll_deg
        self.previous_pitch_command_deg = pitch_deg
        self.last_command_ns = inputs.now_ns
        self._write_demand(
            detected,
            inputs,
            roll_deg=roll_deg,
            pitch_deg=pitch_deg,
            throttle=throttle,
            guided_error_x=guided_error_x,
            guided_error_y=guided_error_y,
        )

    def _release_override(self, reason: str) -> None:
        if self.connection is None or not self.control_authority_active:
            self.control_authority_active = False
            return
        with contextlib.suppress(Exception):
            stt.release_rc_override(self.connection)
        self.control_authority_active = False
        self._write_demand(False, None, failsafe=self.abort_reason is not None, reason=reason)

    def _abort(self, reason: str) -> None:
        self.abort_reason = reason
        self._release_override(reason)

    def _write_demand(
        self,
        detected: bool,
        inputs: ControlInputs | None,
        *,
        roll_deg: float = 0.0,
        pitch_deg: float = 0.0,
        throttle: float = 0.0,
        guided_error_x: float = 0.0,
        guided_error_y: float = 0.0,
        failsafe: bool = False,
        reason: str | None = None,
    ) -> None:
        tracking = None if inputs is None else inputs.tracking
        bbox = None if tracking is None else tracking.bbox
        roll_pwm, pitch_pwm, throttle_pwm, _ = stt.attitude_to_plane_rc_pwm(
            roll_deg,
            pitch_deg,
            self.response_model.max_roll_deg,
            max(self.response_model.max_pitch_up_deg, self.response_model.max_pitch_down_deg),
            throttle,
            roll_rc_reversed=self.response_model.roll_rc_reversed,
            pitch_rc_reversed=self.response_model.pitch_rc_reversed,
            calibration=self.response_model.rc_calibration,
        )
        payload: dict[str, object] = {
            "detected": detected,
            "mode": self.tracking_mode,
            "vehicle": "plane",
            "bbox": list(bbox) if bbox is not None else None,
            "center_x": None if tracking is None else tracking.center_x,
            "center_y": None if tracking is None else tracking.center_y,
            "guided_error_x": guided_error_x,
            "guided_error_y": guided_error_y,
            "roll_deg": roll_deg,
            "pitch_deg": pitch_deg,
            "roll_pwm": roll_pwm,
            "pitch_pwm": pitch_pwm,
            "throttle_pwm": throttle_pwm,
            "max_roll_deg": self.response_model.max_roll_deg,
            "max_pitch_deg": max(
                self.response_model.max_pitch_up_deg,
                self.response_model.max_pitch_down_deg,
            ),
            "rate_hz": self.rate_hz,
            "frame_age_ms": None if inputs is None else inputs.frame_age_ms,
            "tracking_result_age_ms": None if inputs is None else inputs.tracking_result_age_ms,
            "command_age_ms": (
                None
                if inputs is None or self.last_command_ns is None
                else (inputs.now_ns - self.last_command_ns) / 1_000_000.0
            ),
            "tracker_confidence": None if tracking is None else tracking.confidence,
            "tracker_engine": self.tracker_engine_name,
            "vision_state": "TRACK" if detected else "LOST",
            "control_authority": "VULTURE-X" if self.control_authority_active else False,
            "rc_override_active": self.control_authority_active,
            "target_proximity": (
                0.0 if tracking is None else float(tracking.details.get("target_proximity", 0.0))
            ),
            "throttle": throttle,
            "airspeed_mps": None if inputs is None or inputs.vehicle is None else inputs.vehicle.airspeed_mps,
            "effective_max_roll_deg": self.response_model.max_roll_deg,
            "effective_max_pitch_deg": max(
                self.response_model.max_pitch_up_deg,
                self.response_model.max_pitch_down_deg,
            ),
            "tracking_tuning_revision": self.tuning.revision,
            "runtime_metrics": {} if self.runtime is None else self.runtime.metrics.summary(),
            "runtime_active": self.running,
            "failsafe": failsafe,
        }
        if reason is not None:
            payload["failsafe_reason"] = reason
        stt.write_demand_state(DEMAND_STATE_PATH, payload)


def camera_bridge_command(video_source: str, rtsp_url: str) -> list[str]:
    sink = [
        "!",
        "queue",
        "leaky=downstream",
        "max-size-buffers=1",
        "max-size-time=0",
        "max-size-bytes=0",
        "!",
        "videoconvert",
        "!",
        "jpegenc",
        "quality=55",
        "!",
        "multifilesink",
        f"location={CAMERA_DIR}/frame-%06d.jpg",
        "max-files=10",
        "sync=false",
        "async=false",
    ]
    if video_source == "rtsp":
        url = rtsp_url.strip()
        if not url.startswith(("rtsp://", "rtsps://")):
            raise ValueError("rtsp_url_required")
        try:
            latency_ms = max(0, min(500, int(DEFAULT_RTSP_LATENCY_MS)))
        except ValueError as exc:
            raise ValueError("invalid_rtsp_latency_ms") from exc
        protocols = DEFAULT_RTSP_PROTOCOLS.strip().lower()
        if protocols not in {"tcp", "udp"}:
            raise ValueError("invalid_rtsp_protocols")
        try:
            max_rate = max(1, min(60, int(DEFAULT_RTSP_MAX_RATE)))
        except ValueError as exc:
            raise ValueError("invalid_rtsp_max_rate") from exc
        return [
            "gst-launch-1.0",
            "-q",
            "uridecodebin",
            f"uri={url}",
            f"source::latency={latency_ms}",
            "source::drop-on-latency=true",
            "source::do-retransmission=false",
            f"source::protocols={protocols}",
            "!",
            "videorate",
            "drop-only=true",
            "skip-to-first=true",
            f"max-rate={max_rate}",
            *sink,
        ]
    if video_source != "udp":
        raise ValueError("invalid_video_source")
    return [
        "gst-launch-1.0",
        "-q",
        "udpsrc",
        "address=127.0.0.1",
        "port=5600",
        "reuse=false",
        "caps=application/x-rtp,media=video,clock-rate=90000,encoding-name=H264",
        "!",
        "rtph264depay",
        "!",
        "avdec_h264",
        *sink,
    ]


class AppState:
    def __init__(self, profile: VehicleProfile | None = None) -> None:
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        CAMERA_DIR.mkdir(parents=True, exist_ok=True)
        self.profile = profile or VEHICLE_PROFILES[DEFAULT_VEHICLE_MODE]
        self.lock = threading.Lock()
        self.simulator_mode = False
        self.mavlink_endpoint = DEFAULT_MAVLINK_ENDPOINT
        self.video_source = (
            DEFAULT_VIDEO_SOURCE if DEFAULT_VIDEO_SOURCE in {"udp", "rtsp"} else "udp"
        )
        self.rtsp_url = DEFAULT_RTSP_URL
        self.selection_tracker = UiSelectionTracker()
        self.gazebo = ManagedProcess(
            "gazebo",
            ["scripts/run_gazebo.sh", self.profile.gazebo_arg],
            LOG_DIR / "gazebo.log",
            terminal=True,
        )
        self.sitl = ManagedProcess(
            "sitl",
            ["scripts/run_sitl.sh", self.profile.sitl_arg],
            LOG_DIR / "sitl.log",
            {"VULTURE_X_SITL_INTERACTIVE": "0"},
            terminal=True,
        )
        self.takeoff = ManagedProcess(
            "takeoff",
            [
                sys.executable,
                "tools/sitl_arm_takeoff.py",
                "--vehicle",
                self.profile.mode,
                "--altitude-m",
                "50",
            ],
            LOG_DIR / "takeoff.log",
            {"VULTURE_X_ALLOW_SITL_ARM": "1"},
        )
        self.bridge = ManagedProcess(
            "camera bridge",
            camera_bridge_command("udp", ""),
            LOG_DIR / "camera_bridge.log",
        )
        self.target_motion = ManagedProcess(
            "target motion",
            [
                sys.executable,
                "tools/move_gazebo_target.py",
                "--world",
                self.profile.target_world,
                *self.profile.target_motion_args,
            ],
            LOG_DIR / "target_motion.log",
        )
        self.steering = ManagedProcess(
            "steering",
            [
                sys.executable,
                "tools/sitl_track_target.py",
                "--enable-guidance",
                "--camera-dir",
                str(CAMERA_DIR),
            ],
            LOG_DIR / "steering.log",
        )
        self.runtime_steering: RuntimeSteeringSession | None = None
        self.messages: list[str] = []

    def set_mavlink_endpoint(self, endpoint: str) -> str:
        normalized = endpoint.strip()
        try:
            parse_mavlink_endpoint(normalized)
        except ValueError as exc:
            raise ValueError(f"invalid_mavlink_endpoint detail={exc}") from exc
        self.mavlink_endpoint = normalized
        return normalized

    def connect_mavlink(self, endpoint: str | None = None, timeout_s: float = 3.0) -> str:
        endpoint = (endpoint if endpoint is not None else self.mavlink_endpoint).strip()
        try:
            parse_mavlink_endpoint(endpoint)
        except ValueError as exc:
            return f"mavlink blocked reason=invalid_mavlink_endpoint detail={exc}"
        connection = None
        try:
            connection = open_mavlink_connection(
                endpoint,
                source_system=201,
                source_component=203,
                autoreconnect=False,
            )
            send_client_heartbeat(connection)
            heartbeat = connection.wait_heartbeat(timeout=timeout_s)
        except Exception as exc:
            return f"mavlink connect failed reason={exc}"
        finally:
            if connection is not None:
                connection.close()
        if heartbeat is None:
            return "mavlink connect failed reason=heartbeat_timeout"
        self.mavlink_endpoint = endpoint
        mode = mavutil.mode_string_v10(heartbeat)
        return f"mavlink connected endpoint={endpoint} mode={mode}"

    def configure(self, profile: VehicleProfile) -> None:
        self.profile = profile
        self.gazebo.command = ["scripts/run_gazebo.sh", profile.gazebo_arg]
        self.sitl.command = ["scripts/run_sitl.sh", profile.sitl_arg]
        self.takeoff.command = [
            sys.executable,
            "tools/sitl_arm_takeoff.py",
            "--mavlink",
            self.mavlink_endpoint,
            "--vehicle",
            profile.mode,
            "--altitude-m",
            "50" if profile.mode == "plane" else "5",
        ]
        self.target_motion.command = [
            sys.executable,
            "tools/move_gazebo_target.py",
            "--world",
            profile.target_world,
            *profile.target_motion_args,
        ]

    def configure_runtime_io(self, *, simulator: bool) -> None:
        self.simulator_mode = simulator
        if simulator:
            self.mavlink_endpoint = SIM_MAVLINK_ENDPOINT
            self.video_source = SIM_VIDEO_SOURCE if SIM_VIDEO_SOURCE in {"udp", "rtsp"} else "udp"
            self.rtsp_url = ""
            return
        self.mavlink_endpoint = MANUAL_MAVLINK_ENDPOINT
        self.video_source = (
            MANUAL_VIDEO_SOURCE if MANUAL_VIDEO_SOURCE in {"udp", "rtsp"} else "rtsp"
        )
        self.rtsp_url = DEFAULT_RTSP_URL

    def remember(self, message: str) -> None:
        self.messages.append(f"{time.strftime('%H:%M:%S')} {message}")
        self.messages = self.messages[-30:]

    def start_steering(
        self,
        forward_mps: float,
        rate_hz: float,
        max_down_mps: float,
        vertical_gain: float,
        plane_centering_gain: float,
        plane_near_centering_gain: float,
        plane_damping_gain: float,
        plane_near_damping_gain: float,
        plane_far_control_scale: float,
        max_plane_pitch_deg: float,
        plane_pitch_gain_scale: float,
        plane_pitch_near_gain_scale: float,
        plane_pitch_below_center_boost: float,
        plane_pitch_filter_alpha: float,
        plane_max_pitch_step_deg: float,
        plane_max_roll_step_deg: float,
        plane_loss_hold_s: float,
        tracking_mode: str,
        mavlink_endpoint: str | None = None,
        surface_test: bool = False,
        plane_airspeed_mps: float = 20.0,
        max_plane_roll_deg: float = MAX_FIXED_WING_ROLL_DEG,
    ) -> str:
        if mavlink_endpoint is None:
            mavlink_endpoint = self.mavlink_endpoint
        try:
            mavlink_endpoint = self.set_mavlink_endpoint(mavlink_endpoint)
        except ValueError as exc:
            return f"steering blocked reason={exc}"
        if not self.profile.steering_supported:
            return "steering blocked reason=fixed_wing_guidance_not_implemented"
        if surface_test and self.profile.mode != "plane":
            return "steering blocked reason=surface_test_requires_plane"
        forward_mps = max(0.0, min(self.profile.max_forward_mps, forward_mps))
        rate_hz = max(1.0, min(60.0, rate_hz))
        max_down_mps = max(0.0, min(self.profile.max_vertical_mps, max_down_mps))
        vertical_gain = max(0.0, min(self.profile.max_vertical_gain, vertical_gain))
        plane_centering_gain = max(0.0, min(4.0, plane_centering_gain))
        plane_near_centering_gain = max(0.0, min(4.0, plane_near_centering_gain))
        plane_damping_gain = max(0.0, min(3.0, plane_damping_gain))
        plane_near_damping_gain = max(0.0, min(3.0, plane_near_damping_gain))
        plane_far_control_scale = max(0.1, min(1.0, plane_far_control_scale))
        max_plane_roll_deg = max(1.0, min(self.profile.max_roll_deg, max_plane_roll_deg))
        max_plane_pitch_deg = max(0.0, min(self.profile.max_pitch_deg, max_plane_pitch_deg))
        plane_pitch_gain_scale = max(0.0, min(3.0, plane_pitch_gain_scale))
        plane_pitch_near_gain_scale = max(0.0, min(5.0, plane_pitch_near_gain_scale))
        plane_pitch_below_center_boost = max(0.0, min(3.0, plane_pitch_below_center_boost))
        plane_pitch_filter_alpha = max(0.05, min(0.8, plane_pitch_filter_alpha))
        plane_max_pitch_step_deg = max(1.0, min(8.0, plane_max_pitch_step_deg))
        plane_max_roll_step_deg = max(1.0, min(10.0, plane_max_roll_step_deg))
        plane_loss_hold_s = max(0.0, min(5.0, plane_loss_hold_s))
        if surface_test:
            plane_pitch_filter_alpha = max(plane_pitch_filter_alpha, 0.55)
            plane_max_pitch_step_deg = max(plane_max_pitch_step_deg, 6.0)
            plane_max_roll_step_deg = max(plane_max_roll_step_deg, 8.0)
            plane_loss_hold_s = min(plane_loss_hold_s, 0.25)
        elif self.profile.mode == "plane" and self.simulator_mode:
            plane_centering_gain = max(plane_centering_gain, DEFAULT_FIXED_WING_CENTERING_GAIN)
            plane_near_centering_gain = max(
                plane_near_centering_gain,
                DEFAULT_FIXED_WING_NEAR_CENTERING_GAIN,
            )
            plane_damping_gain = min(plane_damping_gain, DEFAULT_FIXED_WING_DAMPING_GAIN)
            plane_near_damping_gain = min(
                plane_near_damping_gain,
                DEFAULT_FIXED_WING_NEAR_DAMPING_GAIN,
            )
            plane_far_control_scale = max(
                plane_far_control_scale,
                DEFAULT_FIXED_WING_FAR_CONTROL_SCALE,
            )
            plane_pitch_gain_scale = max(
                plane_pitch_gain_scale,
                DEFAULT_FIXED_WING_PITCH_GAIN_SCALE,
            )
            plane_pitch_near_gain_scale = max(
                plane_pitch_near_gain_scale,
                DEFAULT_FIXED_WING_PITCH_NEAR_GAIN_SCALE,
            )
            plane_pitch_filter_alpha = max(
                plane_pitch_filter_alpha,
                DEFAULT_FIXED_WING_PITCH_FILTER_ALPHA,
            )
            plane_max_roll_step_deg = max(
                plane_max_roll_step_deg,
                DEFAULT_FIXED_WING_MAX_ROLL_STEP_DEG,
            )
        if tracking_mode == "orange":
            tracking_mode = "red"
        if tracking_mode == "person":
            tracking_mode = "head"
        if tracking_mode not in {"banner", "red", "custom", "head"}:
            return "steering blocked reason=invalid_tracking_mode"
        if tracking_mode in {"custom", "head"} and load_selection() is None:
            return "steering blocked reason=missing_custom_selection"
        with contextlib.suppress(FileNotFoundError):
            DEMAND_STATE_PATH.unlink()
        if self.profile.mode == "plane":
            simulator_throttle_values = (
                {
                    "plane_throttle": DEFAULT_SIM_TRACKING_THROTTLE,
                    "plane_min_throttle": DEFAULT_SIM_TRACKING_THROTTLE,
                    "plane_max_throttle": DEFAULT_SIM_TRACKING_THROTTLE,
                    "plane_lead_s": 0.0,
                    "plane_roll_gain_scale": max(
                        current_tuning_payload()["values"].get("plane_roll_gain_scale", 0.0),
                        1.75,
                    ),
                }
                if self.simulator_mode
                else {}
            )
            tuning_payload = write_tracking_tuning(
                {
                    "plane_airspeed_mps": plane_airspeed_mps,
                    "vertical_gain": vertical_gain,
                    "plane_centering_gain": plane_centering_gain,
                    "plane_near_centering_gain": plane_near_centering_gain,
                    "plane_damping_gain": plane_damping_gain,
                    "plane_near_damping_gain": plane_near_damping_gain,
                    "plane_far_control_scale": plane_far_control_scale,
                    "max_plane_roll_deg": max_plane_roll_deg,
                    "max_plane_pitch_deg": max_plane_pitch_deg,
                    "plane_pitch_gain_scale": plane_pitch_gain_scale,
                    "plane_pitch_near_gain_scale": plane_pitch_near_gain_scale,
                    "plane_pitch_below_center_boost": plane_pitch_below_center_boost,
                    "plane_pitch_filter_alpha": plane_pitch_filter_alpha,
                    "plane_max_pitch_step_deg": plane_max_pitch_step_deg,
                    "plane_max_roll_step_deg": plane_max_roll_step_deg,
                    "plane_loss_hold_s": plane_loss_hold_s,
                    **simulator_throttle_values,
                }
            )
            plane_tuning_values = tuning_payload["values"]
        else:
            plane_tuning_values = {}
        camera_dir = active_camera_dir()
        if camera_dir is None:
            ready_message = self.ensure_camera_ready()
            camera_dir = active_camera_dir()
            if camera_dir is None:
                return f"steering blocked reason=stale_camera detail={ready_message}"
        if self.profile.mode == "plane" and os.environ.get("VULTURE_X_LEGACY_STEERING") != "1":
            if self.runtime_steering is not None and self.runtime_steering.running:
                return "runtime steering already running"
            self.steering.stop()
            runtime_session = RuntimeSteeringSession(
                mavlink_endpoint=mavlink_endpoint,
                video_source=self.video_source,
                rtsp_url=self.rtsp_url,
                camera_dir=camera_dir,
                tracking_mode=tracking_mode,
                rate_hz=rate_hz,
                surface_test=surface_test,
                tuning_values=plane_tuning_values,
                simulator_mode=self.simulator_mode,
            )
            try:
                message = runtime_session.start()
            except Exception as exc:
                runtime_session.close()
                return f"runtime steering blocked reason={type(exc).__name__}"
            if not message.startswith("runtime steering started"):
                return message
            self.runtime_steering = runtime_session
            return message
        self.steering.command = [
            sys.executable,
            "tools/sitl_track_target.py",
            "--enable-guidance",
            "--mavlink",
            mavlink_endpoint,
            "--vehicle",
            self.profile.mode,
            "--source-system",
            "255",
            "--camera-dir",
            str(camera_dir),
            "--forward-mps",
            str(forward_mps),
            "--rate-hz",
            str(rate_hz),
            "--max-down-mps",
            str(max_down_mps),
            "--vertical-gain",
            str(vertical_gain),
            "--tracking-mode",
            tracking_mode,
            "--tracker-engine",
            "nano" if tracking_mode in {"custom", "head"} else "template",
            "--tracker-nano-backbone",
            str(DEFAULT_NANO_BACKBONE),
            "--tracker-nano-neckhead",
            str(DEFAULT_NANO_NECKHEAD),
            "--yunet-model-path",
            str(DEFAULT_YUNET_MODEL),
            "--demand-state-file",
            str(DEMAND_STATE_PATH),
            "--plane-param-cache-file",
            str(PLANE_PARAM_CACHE_PATH),
        ]
        if self.simulator_mode:
            self.steering.command.append("--simulator-mode")
        if self.profile.mode == "plane":
            self.steering.command.extend(
                [
                    "--tuning-file",
                    str(TRACKING_TUNING_PATH),
                    "--max-frame-age-ms",
                    str(DEFAULT_MAX_FRAME_AGE_MS),
                    "--plane-airspeed-mps",
                    str(plane_tuning_values["plane_airspeed_mps"]),
                    "--plane-throttle",
                    str(plane_tuning_values["plane_throttle"]),
                    "--plane-min-throttle",
                    str(plane_tuning_values["plane_min_throttle"]),
                    "--plane-max-throttle",
                    str(plane_tuning_values["plane_max_throttle"]),
                    "--plane-centering-gain",
                    str(plane_tuning_values["plane_centering_gain"]),
                    "--plane-near-centering-gain",
                    str(plane_tuning_values["plane_near_centering_gain"]),
                    "--plane-damping-gain",
                    str(plane_tuning_values["plane_damping_gain"]),
                    "--plane-near-damping-gain",
                    str(plane_tuning_values["plane_near_damping_gain"]),
                    "--plane-far-control-scale",
                    str(plane_tuning_values["plane_far_control_scale"]),
                    "--max-plane-roll-deg",
                    str(plane_tuning_values["max_plane_roll_deg"]),
                    "--max-plane-pitch-deg",
                    str(plane_tuning_values["max_plane_pitch_deg"]),
                    "--plane-pitch-gain-scale",
                    str(plane_tuning_values["plane_pitch_gain_scale"]),
                    "--plane-pitch-near-gain-scale",
                    str(plane_tuning_values["plane_pitch_near_gain_scale"]),
                    "--plane-pitch-below-center-boost",
                    str(plane_tuning_values["plane_pitch_below_center_boost"]),
                    "--plane-pitch-filter-alpha",
                    str(plane_tuning_values["plane_pitch_filter_alpha"]),
                    "--plane-max-pitch-step-deg",
                    str(plane_tuning_values["plane_max_pitch_step_deg"]),
                    "--plane-max-roll-step-deg",
                    str(plane_tuning_values["plane_max_roll_step_deg"]),
                    "--plane-loss-hold-s",
                    str(plane_tuning_values["plane_loss_hold_s"]),
                ]
            )
            if surface_test:
                self.steering.command.append("--surface-test")
                self.steering.command.extend(
                    ["--surface-test-throttle", str(DEFAULT_GROUND_TEST_THROTTLE)]
                )
        if tracking_mode in {"custom", "head"}:
            self.steering.command.extend(["--selection-file", str(SELECTION_PATH)])
        return self.steering.start()

    def ensure_camera_ready(self) -> str:
        if active_camera_dir() is not None:
            return "camera live"
        if camera_bridge_running(self.bridge):
            self.bridge.stop()
        message = self.start_bridge(self.video_source, self.rtsp_url)
        deadline = time.monotonic() + CAMERA_START_WAIT_S
        while time.monotonic() < deadline:
            if active_camera_dir() is not None:
                return f"camera restarted detail={message}"
            time.sleep(0.1)
        return f"camera restart timeout detail={message}"

    def start_takeoff(self, mavlink_endpoint: str | None = None) -> str:
        if mavlink_endpoint is None:
            mavlink_endpoint = self.mavlink_endpoint
        try:
            mavlink_endpoint = self.set_mavlink_endpoint(mavlink_endpoint)
        except ValueError as exc:
            return f"takeoff blocked reason={exc}"
        if self.profile.mode != "plane":
            return "takeoff blocked reason=plane_profile_required"
        if not self.simulator_mode:
            return "takeoff blocked reason=simulator_only"
        if self.takeoff.running():
            return "takeoff already running"
        self.takeoff.command = [
            sys.executable,
            "tools/sitl_arm_takeoff.py",
            "--mavlink",
            mavlink_endpoint,
            "--vehicle",
            self.profile.mode,
            "--altitude-m",
            "50",
        ]
        return self.takeoff.start()

    def set_plane_auto(self, mavlink_endpoint: str | None = None) -> str:
        if self.profile.mode != "plane":
            return "auto skipped reason=plane_profile_required"
        if not self.simulator_mode:
            return "auto skipped reason=simulator_only"
        endpoint = mavlink_endpoint if mavlink_endpoint is not None else self.mavlink_endpoint
        connection = None
        try:
            connection = open_mavlink_connection(
                endpoint,
                source_system=255,
                source_component=203,
                autoreconnect=False,
            )
            send_client_heartbeat(connection)
            heartbeat = connection.wait_heartbeat(timeout=3.0)
            if heartbeat is None:
                return "auto failed reason=heartbeat_timeout"
            auto_mode = connection.mode_mapping().get("AUTO")
            if auto_mode is None:
                return "auto failed reason=auto_mode_unavailable"
            connection.set_mode(auto_mode)
            return "auto commanded"
        except Exception as exc:
            return f"auto failed reason={type(exc).__name__}"
        finally:
            if connection is not None:
                connection.close()

    def stop_steering(self) -> str:
        if self.runtime_steering is not None and self.runtime_steering.running:
            stop_message = self.runtime_steering.stop(
                request_auto=self.profile.mode == "plane" and self.simulator_mode
            )
            self.runtime_steering = None
            return stop_message
        stop_message = self.steering.stop()
        auto_message = self.set_plane_auto()
        if auto_message.startswith("auto skipped"):
            return stop_message
        return f"{stop_message}; {auto_message}"

    def start_gazebo(self) -> str:
        if process_running(self.profile.gazebo_process_pattern):
            return "gazebo already running"
        stop_stale_gazebo_server()
        return self.gazebo.start()

    def start_sitl(self) -> str:
        if process_running(self.profile.sitl_process_pattern):
            return "sitl already running"
        stop_stale_mavproxy()
        return self.sitl.start()

    def start_bridge(self, video_source: str | None = None, rtsp_url: str | None = None) -> str:
        video_source = (video_source or self.video_source).strip()
        rtsp_url = (rtsp_url if rtsp_url is not None else self.rtsp_url).strip()
        try:
            self.bridge.command = camera_bridge_command(video_source, rtsp_url)
        except ValueError as exc:
            return f"camera bridge blocked reason={exc}"
        self.video_source = video_source
        self.rtsp_url = rtsp_url
        conflicts = udp_port_conflicts(5600) if video_source == "udp" else []
        if conflicts:
            return "camera bridge blocked reason=udp_5600_in_use owners=" + " | ".join(conflicts)
        if camera_bridge_running(self.bridge):
            return "camera bridge already running"
        CAMERA_DIR.mkdir(parents=True, exist_ok=True)
        clear_camera_frames()
        self.selection_tracker.reset()
        message = self.bridge.start()
        if video_source == "udp":
            schedule_camera_streaming_retries()
        return message

    def start_target_motion(self) -> str:
        if process_running(r"move_gazebo_target.py"):
            return "target motion already running"
        return self.target_motion.start()

    def auto_start(self) -> list[str]:
        messages = [
            self.start_gazebo(),
            self.start_sitl(),
            self.start_bridge(),
        ]
        if self.profile.autostart_target_motion:
            messages.append(self.start_target_motion())
        else:
            messages.append("target motion skipped reason=static_plane_banner")
        return messages

    def stop_all(self) -> list[str]:
        if self.runtime_steering is not None:
            self.runtime_steering.stop(request_auto=False)
            self.runtime_steering = None
        messages = [
            self.steering.stop(),
            self.takeoff.stop(),
            self.target_motion.stop(),
            self.bridge.stop(),
            self.sitl.stop(),
            self.gazebo.stop(),
        ]
        for pattern in (
            r"move_gazebo_target.py",
            r"gst-launch-1.0 .*port=5600",
            r"gst-launch-1.0 .*rtspsrc",
            r"gst-launch-1.0 .*uridecodebin .*rtsp",
            r"arducopter --model JSON",
            r"arduplane --model JSON",
            r"sim_vehicle.py .*gazebo-iris",
            r"sim_vehicle.py .*gazebo-zephyr",
            r"mavproxy.py .*--master tcp:127\.0\.0\.1:5760",
            r"gz sim .*vulture_x_test",
            r"gz sim .*vulture_x_plane",
            r"^gz sim gui",
            r"^gz sim server",
        ):
            stop_matching_processes(pattern)
        return messages


STATE = AppState()
MAVLINK_MONITOR = MavlinkStatusMonitor(MAVLINK_STATUS_ENDPOINT)


def terminal_launcher(name: str, command: list[str], log_path: Path) -> list[str] | None:
    terminal = (
        shutil.which("x-terminal-emulator")
        or shutil.which("gnome-terminal")
        or shutil.which("konsole")
        or shutil.which("xfce4-terminal")
        or shutil.which("mate-terminal")
    )
    if terminal is None:
        return None
    quoted = " ".join(shlex.quote(part) for part in command)
    quoted_log_path = shlex.quote(str(log_path))
    shell = (
        f"cd {shlex.quote(str(REPO_ROOT))}; "
        f"{quoted} 2>&1 | tee -a {quoted_log_path}; "
        "status=${PIPESTATUS[0]}; "
        "echo; echo \"Process exited with status ${status}. See log for details.\"; "
        "exit ${status}"
    )
    title = f"Vulture-X {name}"
    executable = Path(terminal).name
    if executable == "gnome-terminal":
        return [terminal, "--title", title, "--", "bash", "-lc", shell]
    if executable == "konsole":
        return [terminal, "--new-tab", "-p", f"tabtitle={title}", "-e", "bash", "-lc", shell]
    if executable == "xfce4-terminal":
        return [terminal, "--title", title, "--command", f"bash -lc {shlex.quote(shell)}"]
    if executable == "mate-terminal":
        return [terminal, "--title", title, "--", "bash", "-lc", shell]
    return [terminal, "-T", title, "-e", "bash", "-lc", shell]


def process_running(pattern: str) -> bool:
    result = subprocess.run(
        ["pgrep", "-f", pattern],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return result.returncode == 0


def camera_bridge_running(bridge: ManagedProcess) -> bool:
    return (
        bridge.running()
        or process_running(r"gst-launch-1.0 .*port=5600")
        or process_running(r"gst-launch-1.0 .*rtspsrc")
        or process_running(r"gst-launch-1.0 .*uridecodebin .*rtsp")
    )


def stop_matching_processes(pattern: str) -> None:
    subprocess.run(
        ["pkill", "-TERM", "-f", pattern],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )


def kill_matching_processes(pattern: str) -> None:
    subprocess.run(
        ["pkill", "-KILL", "-f", pattern],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )


def stop_stale_gazebo_server() -> None:
    stop_matching_processes(r"^gz sim( |$)")
    time.sleep(1)
    kill_matching_processes(r"^gz sim( |$)")


def stop_stale_mavproxy() -> None:
    stop_matching_processes(r"mavproxy.py .*--master tcp:127\.0\.0\.1:5760")
    time.sleep(0.5)
    kill_matching_processes(r"mavproxy.py .*--master tcp:127\.0\.0\.1:5760")


def udp_port_owners(port: int) -> list[str]:
    result = subprocess.run(
        ["ss", "-lunp"],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        check=False,
    )
    owners: list[str] = []
    needle = f":{port} "
    for line in result.stdout.splitlines():
        if needle in line and "users:" in line:
            owners.append(line.strip())
    return owners


def udp_port_conflicts(port: int) -> list[str]:
    conflicts = []
    for owner in udp_port_owners(port):
        if "gst-launch-1.0" not in owner:
            conflicts.append(owner)
    return conflicts


def clear_camera_frames() -> int:
    removed = 0
    for image_path in CAMERA_DIR.glob("frame-*"):
        try:
            image_path.unlink()
            removed += 1
        except FileNotFoundError:
            continue
    return removed


def load_selection() -> dict[str, float | bool | str] | None:
    if not SELECTION_PATH.exists():
        return None
    try:
        raw = json.loads(SELECTION_PATH.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    mode = "head" if raw.get("mode") == "person" else raw.get("mode")
    if mode not in {"custom", "head"}:
        return None
    try:
        x = float(raw["x"])
        y = float(raw["y"])
        width = float(raw["width"])
        height = float(raw["height"])
    except (KeyError, TypeError, ValueError):
        return None
    if x < 0 or y < 0 or width <= 0 or height <= 0 or x + width > 1 or y + height > 1:
        return None
    return {
        "enabled": True,
        "mode": str(mode),
        "x": x,
        "y": y,
        "width": width,
        "height": height,
    }


def selection_payload() -> dict[str, float | bool | str]:
    selection = load_selection()
    if selection is None:
        return {"enabled": False, "mode": "red", "x": 0.0, "y": 0.0, "width": 0.0, "height": 0.0}
    tracked = STATE.selection_tracker.payload()
    return tracked or {**selection, "tracking": "initializing"}


def normalized_selection_to_bbox(
    selection: dict[str, float | bool | str],
    frame: Any,
) -> tuple[int, int, int, int]:
    frame_height, frame_width = frame.shape[:2]
    x = round(float(selection["x"]) * frame_width)
    y = round(float(selection["y"]) * frame_height)
    width = round(float(selection["width"]) * frame_width)
    height = round(float(selection["height"]) * frame_height)
    x = round(max(0, min(frame_width - 1, x)))
    y = round(max(0, min(frame_height - 1, y)))
    width = round(max(1, min(frame_width - x, width)))
    height = round(max(1, min(frame_height - y, height)))
    return x, y, width, height


def create_cv_tracker(tracker_name: str) -> Any:
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


def save_selection(raw: dict[str, Any]) -> str:
    try:
        x = float(raw["x"])
        y = float(raw["y"])
        width = float(raw["width"])
        height = float(raw["height"])
    except (KeyError, TypeError, ValueError):
        return "selection rejected reason=invalid_payload"
    if width < 0.02 or height < 0.02:
        return "selection rejected reason=selection_too_small"
    if x < 0 or y < 0 or x + width > 1 or y + height > 1:
        return "selection rejected reason=selection_outside_frame"
    mode = "head" if raw.get("mode", "custom") == "person" else raw.get("mode", "custom")
    if mode not in {"custom", "head"}:
        return "selection rejected reason=invalid_mode"
    payload = {
        "mode": mode,
        "x": x,
        "y": y,
        "width": width,
        "height": height,
        "updated_at_unix_s": time.time(),
    }
    SELECTION_PATH.write_text(json.dumps(payload, sort_keys=True), encoding="utf-8")
    STATE.selection_tracker.reset()
    return f"{mode} selection saved"


def current_camera_frame() -> tuple[Any | None, Path | None]:
    camera_dir = active_camera_dir()
    if camera_dir is None:
        return None, None
    image_path = newest_stable_image(camera_dir)
    if image_path is None:
        return None, None
    return read_camera_frame(image_path), image_path


def cached_heads(frame: Any, image_path: Path) -> list[tuple[int, int, int, int]]:
    try:
        frame_key = (str(image_path), image_path.stat().st_mtime_ns)
    except FileNotFoundError:
        return []
    if HEAD_DETECTION_CACHE.get("key") == frame_key:
        cached = HEAD_DETECTION_CACHE.get("heads")
        if isinstance(cached, list):
            return cached
    heads = detect_heads_yunet(frame)
    HEAD_DETECTION_CACHE["key"] = frame_key
    HEAD_DETECTION_CACHE["heads"] = heads
    return heads


def detect_heads_yunet(frame: Any) -> list[tuple[int, int, int, int]]:
    if not DEFAULT_YUNET_MODEL.exists() or not hasattr(cv2, "FaceDetectorYN_create"):
        return detect_heads(frame, max_width=HEAD_DETECTION_MAX_WIDTH)
    frame_height, frame_width = frame.shape[:2]
    input_width, input_height = YUNET_INPUT_SIZE
    resized = cv2.resize(frame, (input_width, input_height), interpolation=cv2.INTER_AREA)
    detector = cv2.FaceDetectorYN_create(
        str(DEFAULT_YUNET_MODEL),
        "",
        (input_width, input_height),
        YUNET_SCORE_THRESHOLD,
        0.3,
        20,
    )
    _retval, faces = detector.detect(resized)
    if faces is None:
        return []
    scale_x = frame_width / input_width
    scale_y = frame_height / input_height
    heads: list[tuple[int, int, int, int]] = []
    for face in faces:
        x, y, width, height = (float(face[index]) for index in range(4))
        heads.append(
            clamp_bbox(
                (
                    round(x * scale_x),
                    round(y * scale_y),
                    round(width * scale_x),
                    round(height * scale_y),
                ),
                frame_width,
                frame_height,
            )
        )
    return heads


def normalized_bbox_from_pixels(
    bbox: tuple[int, int, int, int],
    frame: Any,
) -> dict[str, float | str]:
    frame_height, frame_width = frame.shape[:2]
    x, y, width, height = bbox
    return {
        "mode": "head",
        "x": x / frame_width,
        "y": y / frame_height,
        "width": width / frame_width,
        "height": height / frame_height,
    }


def select_head_at(x_norm: float, y_norm: float) -> str:
    if not math.isfinite(x_norm) or not math.isfinite(y_norm):
        return "head selection rejected reason=invalid_point"
    if x_norm < 0 or y_norm < 0 or x_norm > 1 or y_norm > 1:
        return "head selection rejected reason=point_outside_frame"
    frame, _image_path = current_camera_frame()
    if frame is None:
        return "head selection rejected reason=no_camera_frame"
    frame_height, frame_width = frame.shape[:2]
    point_x = x_norm * frame_width
    point_y = y_norm * frame_height
    heads = cached_heads(frame, _image_path) if _image_path is not None else []
    if not heads:
        fallback_width = max(24, round(frame_width * 0.08))
        fallback_height = max(24, round(fallback_width * 1.15))
        chosen = (
            round(point_x - fallback_width / 2.0),
            round(point_y - fallback_height / 2.0),
            fallback_width,
            fallback_height,
        )
        chosen = clamp_bbox(chosen, frame_width, frame_height)
        message = save_selection(normalized_bbox_from_pixels(chosen, frame))
        if message.endswith("selection saved"):
            return "head selection saved heads_detected=0 fallback=click_roi"
        return message

    containing = [
        bbox
        for bbox in heads
        if bbox[0] <= point_x <= bbox[0] + bbox[2] and bbox[1] <= point_y <= bbox[1] + bbox[3]
    ]
    if containing:
        chosen = min(containing, key=lambda bbox: bbox[2] * bbox[3])
    else:
        frame_diag = max(
            1.0,
            float((frame_width * frame_width + frame_height * frame_height) ** 0.5),
        )
        scored = []
        for bbox in heads:
            center_x = bbox[0] + bbox[2] / 2.0
            center_y = bbox[1] + bbox[3] / 2.0
            distance = ((center_x - point_x) ** 2 + (center_y - point_y) ** 2) ** 0.5
            scored.append((distance / frame_diag, bbox))
        distance_norm, chosen = min(scored, key=lambda item: item[0])
        if distance_norm > 0.12:
            return "head selection rejected reason=no_head_near_click"

    message = save_selection(normalized_bbox_from_pixels(chosen, frame))
    if message.endswith("selection saved"):
        return f"{message} heads_detected={len(heads)}"
    return message


def enable_gazebo_camera_streaming() -> None:
    result = subprocess.run(
        ["gz", "topic", "-l"],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        check=False,
    )
    topics = [
        line.strip()
        for line in result.stdout.splitlines()
        if line.strip().endswith("/enable_streaming")
    ] or [
        "/world/vulture_x_test/model/iris_with_gimbal/model/gimbal/link/"
        "pitch_link/sensor/camera/image/enable_streaming",
        "/world/vulture_x_plane/model/zephyr_with_camera/link/"
        "fixed_wing_camera_link/sensor/camera/image/enable_streaming",
    ]
    for topic in topics:
        subprocess.run(
            ["gz", "topic", "-t", topic, "-m", "gz.msgs.Boolean", "-p", "data: true"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            text=True,
            check=False,
        )


def schedule_camera_streaming_retries() -> None:
    for delay_s in CAMERA_STREAM_ENABLE_RETRY_S:
        threading.Timer(delay_s, enable_gazebo_camera_streaming).start()


def active_camera_dir() -> Path | None:
    candidates = [CAMERA_DIR]
    candidates.extend(sorted(Path("/tmp").glob("vulture-x-target-camera-*")))
    candidates.extend(sorted(Path("/tmp").glob("vulture-x-camera-*")))
    newest: tuple[float, Path] | None = None
    now = time.time()
    for directory in candidates:
        if not directory.is_dir():
            continue
        image_path = newest_image(directory)
        if image_path is None:
            continue
        try:
            mtime = image_path.stat().st_mtime
        except FileNotFoundError:
            continue
        if now - mtime > FRESH_FRAME_MAX_AGE_S:
            continue
        if newest is None or mtime > newest[0]:
            newest = (mtime, directory)
    return newest[1] if newest else None


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
    if not candidates:
        return newest_image(camera_dir)
    return max(candidates, key=lambda item: item[0])[1]


def read_camera_frame(image_path: Path) -> Any | None:
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


def mavlink_status() -> dict[str, Any]:
    status = MAVLINK_MONITOR.snapshot()
    status["control_endpoint"] = STATE.mavlink_endpoint
    return status


def normalize_hud_mode(value: str) -> str:
    return value if value in {"operational", "diagnostic"} else "operational"


def hud_enabled_from_query(value: str) -> bool:
    return value not in {"0", "false", "False", "off", "OFF"}


def send_client_heartbeat(connection: mavutil.mavfile) -> None:
    connection.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        0,
    )


def latest_runtime_target(
    *,
    hud_mode: str = "operational",
    hud_enabled: bool = True,
) -> tuple[dict[str, Any], bytes | None] | None:
    session = STATE.runtime_steering
    if session is None or not session.running:
        return None
    snapshot = session.snapshot()
    if snapshot is None or snapshot.frame is None:
        return {"detected": False, "detail": "runtime frame unavailable"}, None
    tracking = snapshot.tracking
    demand = load_demand_state()
    if tracking is None:
        target: dict[str, Any] = {"detected": False, "detail": "runtime tracking unavailable"}
    elif tracking.detected and tracking.bbox is not None:
        target = {
            "detected": True,
            "mode": tracking.mode,
            "bbox": list(tracking.bbox),
            "center_x": tracking.center_x,
            "center_y": tracking.center_y,
            "runtime": True,
        }
    else:
        target = {
            "detected": False,
            "mode": "runtime",
            "detail": "runtime tracking lost",
            "runtime": True,
        }

    def render(image: Any, runtime_snapshot: RuntimeSnapshot) -> None:
        del runtime_snapshot
        render_hud(image, target, demand, mavlink_status(), [], normalize_hud_mode(hud_mode))

    image = encode_runtime_preview_jpeg(
        snapshot.frame,
        snapshot,
        hud_renderer=render if hud_enabled else None,
    )
    return target, image


def latest_target(
    display_mode: str = "red",
    *,
    hud_mode: str = "operational",
    hud_enabled: bool = True,
) -> tuple[dict[str, Any], bytes | None]:
    runtime_target = latest_runtime_target(hud_mode=hud_mode, hud_enabled=hud_enabled)
    if runtime_target is not None:
        return runtime_target
    if display_mode not in {"red", "banner", "custom", "head", "person"}:
        display_mode = "red"
    camera_dir = active_camera_dir()
    if camera_dir is None:
        return {"detected": False, "detail": "no camera frame"}, None
    image_path = newest_image(camera_dir)
    if image_path is None:
        return {"detected": False, "detail": "no camera frame"}, None
    image_path = newest_stable_image(camera_dir)
    if image_path is None:
        return {"detected": False, "detail": "no stable camera frame"}, None
    frame = read_camera_frame(image_path)
    if frame is None:
        return {"detected": False, "detail": "frame unreadable"}, None
    overlay_start_ns = time.monotonic_ns()
    saved_selection = load_selection()
    selection = None if display_mode == "banner" else saved_selection
    heads: list[tuple[int, int, int, int]] = []
    demand = load_demand_state()
    if selection is not None:
        bbox = None
        if (
            demand is not None
            and STATE.steering.running()
            and demand.get("mode") in {"custom", "head", "person"}
            and demand.get("detected")
        ):
            bbox = demand_bbox(demand, frame)
        if bbox is None:
            bbox = STATE.selection_tracker.bbox(frame, image_path)
        if bbox is None:
            bbox = normalized_selection_to_bbox(selection, frame)
    elif display_mode in {"head", "person"}:
        heads = cached_heads(frame, image_path)
        bbox = None
    elif display_mode == "banner":
        bbox = detect_banner_target(frame, 25.0)
    else:
        bbox = detect_colored_target(frame, 25.0)
    target: dict[str, Any] = {"detected": False, "detail": "not detected"}
    if bbox is not None:
        x, y, width, height = bbox
        frame_height, frame_width = frame.shape[:2]
        center_x = (x + width / 2) / frame_width
        center_y = (y + height / 2) / frame_height
        mode = str(selection.get("mode", "custom")) if selection is not None else "red"
        if selection is None:
            mode = display_mode
        target = {
            "detected": True,
            "mode": mode,
            "bbox": [x, y, width, height],
            "center_x": center_x,
            "center_y": center_y,
            "camera_dir": str(camera_dir),
        }
    elif selection is not None:
        target = {
            "detected": False,
            "mode": str(selection.get("mode", "custom")),
            "detail": "custom tracking lost",
        }
        frame_height, frame_width = frame.shape[:2]
        sx = int(float(selection["x"]) * frame_width)
        sy = int(float(selection["y"]) * frame_height)
        sw = int(float(selection["width"]) * frame_width)
        sh = int(float(selection["height"]) * frame_height)
        target["reference_bbox"] = [sx, sy, sw, sh]
    elif display_mode in {"head", "person"}:
        target = {
            "detected": False,
            "mode": "head",
            "detail": "select a head/face",
            "head_count": len(heads),
        }
    if hud_enabled:
        render_hud(frame, target, demand, mavlink_status(), heads, normalize_hud_mode(hud_mode))
    overlay_ms = (time.monotonic_ns() - overlay_start_ns) / 1_000_000.0
    target["overlay_render_ms"] = overlay_ms
    ok, encoded = cv2.imencode(".jpg", frame)
    return target, encoded.tobytes() if ok else None


def load_demand_state(max_age_s: float = 1.5) -> dict[str, Any] | None:
    try:
        payload = json.loads(DEMAND_STATE_PATH.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    updated_unix_s = payload.get("updated_unix_s")
    if not isinstance(updated_unix_s, (int, float)):
        return None
    if time.time() - float(updated_unix_s) > max_age_s:
        return None
    return payload


def demand_float(payload: dict[str, Any], key: str, default: float = 0.0) -> float:
    value = payload.get(key, default)
    if isinstance(value, (int, float)):
        return float(value)
    return default


def demand_bbox(payload: dict[str, Any], frame: Any) -> tuple[int, int, int, int] | None:
    raw_bbox = payload.get("bbox")
    if not isinstance(raw_bbox, (list, tuple)) or len(raw_bbox) != 4:
        return None
    try:
        bbox = tuple(round(float(value)) for value in raw_bbox)
    except (TypeError, ValueError):
        return None
    frame_height, frame_width = frame.shape[:2]
    return clamp_bbox(bbox, frame_width, frame_height)


def hud_scale(frame: Any) -> float:
    frame_height, frame_width = frame.shape[:2]
    return max(0.75, min(2.2, min(frame_width / 1280.0, frame_height / 720.0)))


def semantic_color(state: str) -> tuple[int, int, int]:
    if state == "green":
        return (90, 230, 120)
    if state == "amber":
        return (0, 210, 255)
    if state == "red":
        return (80, 90, 255)
    return (245, 245, 245)


def outlined_text(
    frame: Any,
    text: str,
    origin: tuple[int, int],
    *,
    scale: float,
    color: tuple[int, int, int] = (245, 245, 245),
    thickness: int = 1,
) -> None:
    cv2.putText(
        frame,
        text,
        origin,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        (0, 0, 0),
        thickness + 2,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        text,
        origin,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        thickness,
        cv2.LINE_AA,
    )


def draw_corner_brackets(
    frame: Any,
    bbox: tuple[int, int, int, int],
    color: tuple[int, int, int],
    *,
    label: str | None = None,
) -> None:
    x, y, width, height = bbox
    scale = hud_scale(frame)
    thickness = max(1, round(2 * scale))
    shadow = (0, 0, 0)
    corner = round(max(10.0 * scale, min(width * 0.24, height * 0.24, 34.0 * scale)))
    points = [
        ((x, y), (x + corner, y)),
        ((x, y), (x, y + corner)),
        ((x + width, y), (x + width - corner, y)),
        ((x + width, y), (x + width, y + corner)),
        ((x, y + height), (x + corner, y + height)),
        ((x, y + height), (x, y + height - corner)),
        ((x + width, y + height), (x + width - corner, y + height)),
        ((x + width, y + height), (x + width, y + height - corner)),
    ]
    for start, end in points:
        cv2.line(frame, start, end, shadow, thickness + 2, cv2.LINE_AA)
        cv2.line(frame, start, end, color, thickness, cv2.LINE_AA)
    if label:
        outlined_text(
            frame,
            label,
            (x, max(18, y - round(8 * scale))),
            scale=0.46 * scale,
            color=color,
            thickness=thickness,
        )


def command_cue_point(
    frame: Any,
    demand: dict[str, Any] | None,
) -> tuple[int, int]:
    frame_height, frame_width = frame.shape[:2]
    center_x = frame_width // 2
    center_y = frame_height // 2
    if demand is None:
        return center_x, center_y
    roll_deg = demand_float(demand, "roll_deg")
    pitch_deg = demand_float(demand, "pitch_deg")
    max_roll_deg = max(1.0, demand_float(demand, "max_roll_deg", MAX_FIXED_WING_ROLL_DEG))
    max_pitch_deg = max(1.0, demand_float(demand, "max_pitch_deg", MAX_FIXED_WING_PITCH_DEG))
    if abs(roll_deg) < 0.4:
        roll_deg = 0.0
    if abs(pitch_deg) < 0.4:
        pitch_deg = 0.0
    cue_radius_x = frame_width * 0.22
    cue_radius_y = frame_height * 0.22
    cue_x = center_x + round(max(-1.0, min(1.0, roll_deg / max_roll_deg)) * cue_radius_x)
    cue_y = center_y - round(max(-1.0, min(1.0, pitch_deg / max_pitch_deg)) * cue_radius_y)
    return max(0, min(frame_width - 1, cue_x)), max(0, min(frame_height - 1, cue_y))


def draw_center_overlay(frame: Any) -> None:
    frame_height, frame_width = frame.shape[:2]
    center_x = frame_width // 2
    center_y = frame_height // 2
    scale = hud_scale(frame)
    gap = round(13 * scale)
    length = round(18 * scale)
    thickness = max(1, round(scale))
    color = (245, 245, 245)
    shadow = (0, 0, 0)
    segments = [
        ((center_x - gap - length, center_y), (center_x - gap, center_y)),
        ((center_x + gap, center_y), (center_x + gap + length, center_y)),
        ((center_x, center_y - gap - length), (center_x, center_y - gap)),
        ((center_x, center_y + gap), (center_x, center_y + gap + length)),
    ]
    for start, end in segments:
        cv2.line(frame, start, end, shadow, thickness + 2, cv2.LINE_AA)
        cv2.line(frame, start, end, color, thickness, cv2.LINE_AA)
    cv2.drawMarker(
        frame,
        (center_x, center_y),
        shadow,
        cv2.MARKER_CROSS,
        round(8 * scale),
        thickness + 2,
        cv2.LINE_AA,
    )
    cv2.drawMarker(
        frame,
        (center_x, center_y),
        color,
        cv2.MARKER_CROSS,
        round(8 * scale),
        thickness,
        cv2.LINE_AA,
    )


def vision_state(target: dict[str, Any], demand: dict[str, Any] | None) -> tuple[str, str]:
    if demand and demand.get("failsafe"):
        reason = str(demand.get("failsafe_reason", "released")).replace("_", " ").upper()
        return "TARGET LOST" if reason == "TARGET LOST" else reason, "red"
    if demand and isinstance(demand.get("vision_state"), str):
        state = str(demand["vision_state"]).upper()
        if state == "TRACK":
            return "TRACK LOCK", "green"
        if state == "DEGRADED":
            return "TRACK DEGRADED", "amber"
        if state == "LOST":
            return "TARGET LOST", "red"
    if target.get("detected"):
        return "TARGET READY", "amber"
    return "TARGET ACQUIRING", "amber"


def control_authority_text(demand: dict[str, Any] | None) -> str:
    if demand is None:
        return "CTRL AUTOPILOT"
    if demand.get("rc_override_active"):
        return "CTRL VULTURE-X"
    if demand.get("failsafe"):
        return "CTRL PILOT"
    return str(demand.get("control_authority", "CTRL AUTOPILOT"))


def confidence_text(demand: dict[str, Any] | None) -> str:
    confidence = demand.get("tracker_confidence") if demand else None
    if not isinstance(confidence, (int, float)):
        return "CONF --"
    return f"CONF {max(0.0, min(1.0, float(confidence))) * 100.0:.0f}%"


def age_text(demand: dict[str, Any] | None) -> str:
    if demand is None:
        return "AGE --"
    age = demand.get("tracking_result_age_ms", demand.get("frame_age_ms"))
    if not isinstance(age, (int, float)):
        return "AGE --"
    return f"AGE {float(age):.0f} ms"


def draw_top_bar(
    frame: Any,
    state_label: str,
    state_style: str,
    mavlink: dict[str, Any],
) -> None:
    scale = hud_scale(frame)
    mode = str(mavlink.get("mode", "offline")).upper()
    link = "LINK" if mavlink.get("connected") else "LINK LOST"
    color = semantic_color(state_style)
    text = f"VULTURE-X     {state_label}     {mode}     {link}"
    outlined_text(frame, text, (round(16 * scale), round(28 * scale)), scale=0.58 * scale, color=color, thickness=max(1, round(2 * scale)))


def draw_bottom_strip(frame: Any, demand: dict[str, Any] | None) -> None:
    frame_height, _frame_width = frame.shape[:2]
    scale = hud_scale(frame)
    alt = demand.get("relative_alt_m") if demand else None
    speed = demand.get("airspeed_mps") if demand else None
    alt_text = f"ALT {float(alt):.0f} m" if isinstance(alt, (int, float)) else "ALT --"
    speed_text = f"SPD {float(speed):.1f} m/s" if isinstance(speed, (int, float)) else "SPD --"
    text = f"{alt_text}     {speed_text}     {confidence_text(demand)}     {age_text(demand)}"
    outlined_text(frame, text, (round(16 * scale), frame_height - round(18 * scale)), scale=0.52 * scale, thickness=max(1, round(2 * scale)))


def render_hud(
    frame: Any,
    target: dict[str, Any],
    demand: dict[str, Any] | None,
    mavlink: dict[str, Any],
    heads: list[tuple[int, int, int, int]],
    hud_mode: str,
) -> None:
    state_label, state_style = vision_state(target, demand)
    color = semantic_color(state_style)
    draw_top_bar(frame, state_label, state_style, mavlink)
    if target.get("detected") and "bbox" in target:
        draw_corner_brackets(
            frame,
            tuple(int(value) for value in target["bbox"]),
            color,
            label="LOCK" if state_style == "green" else "ACQ",
        )
    elif "reference_bbox" in target and hud_mode == "diagnostic":
        draw_corner_brackets(
            frame,
            tuple(int(value) for value in target["reference_bbox"]),
            semantic_color("amber"),
            label="REF",
        )
    for index, head_bbox in enumerate(heads, start=1):
        draw_corner_brackets(frame, head_bbox, semantic_color("amber"), label=str(index))
    draw_center_overlay(frame)
    cue = command_cue_point(frame, demand)
    cv2.circle(frame, cue, max(5, round(6 * hud_scale(frame))), (0, 0, 0), 3, cv2.LINE_AA)
    cv2.circle(frame, cue, max(4, round(5 * hud_scale(frame))), semantic_color("amber"), 1, cv2.LINE_AA)
    outlined_text(frame, "CMD", (cue[0] + 8, cue[1] + 18), scale=0.36 * hud_scale(frame), color=semantic_color("amber"))
    outlined_text(frame, control_authority_text(demand), (round(16 * hud_scale(frame)), round(52 * hud_scale(frame))), scale=0.42 * hud_scale(frame), color=semantic_color("green" if demand and demand.get("rc_override_active") else "amber"))
    draw_bottom_strip(frame, demand)
    if hud_mode == "diagnostic":
        draw_diagnostic_hud(frame, target, demand)


def draw_diagnostic_hud(
    frame: Any,
    target: dict[str, Any],
    demand: dict[str, Any] | None,
) -> None:
    scale = hud_scale(frame)
    metrics = demand.get("runtime_metrics", {}) if demand else {}
    lines = [
        "TRACKER Template",
        f"STATE {vision_state(target, demand)[0]}",
        confidence_text(demand),
        f"VISION {metric_text(metrics, 'vision_hz', 'Hz')} {metric_text(metrics, 'vision_ms_p95', 'ms P95')}",
        f"CONTROL {metric_text(metrics, 'control_hz', 'Hz')} JIT {metric_text(metrics, 'control_jitter_ms_p95', 'ms P95')}",
        f"MAV TX {metric_text(metrics, 'mavlink_tx_hz', 'Hz')}",
    ]
    if demand:
        lines.extend(
            [
                f"ROLL {demand_float(demand, 'roll_deg'):+.1f} PITCH {demand_float(demand, 'pitch_deg'):+.1f}",
                f"THR {demand_float(demand, 'throttle'):.2f}",
                f"PWM R {round(demand_float(demand, 'roll_pwm', 1500.0))} P {round(demand_float(demand, 'pitch_pwm', 1500.0))} T {round(demand_float(demand, 'throttle_pwm', 1000.0))}",
                f"PROX {demand_float(demand, 'target_proximity'):.2f}",
                f"DROPS {metrics.get('dropped_frames', 0)} MISS {metrics.get('deadline_misses', 0)} FAIL {metrics.get('tracker_failures', 0)}",
            ]
        )
    draw_text_block(frame, lines, (round(16 * scale), round(80 * scale)))


def metric_text(metrics: object, key: str, suffix: str) -> str:
    if not isinstance(metrics, dict):
        return "--"
    value = metrics.get(key)
    if not isinstance(value, (int, float)):
        return "--"
    return f"{float(value):.1f}{suffix}"


def draw_text_block(frame: Any, lines: list[str], origin: tuple[int, int]) -> None:
    scale = hud_scale(frame)
    line_height = round(18 * scale)
    for index, line in enumerate(lines):
        outlined_text(
            frame,
            line,
            (origin[0], origin[1] + index * line_height),
            scale=0.42 * scale,
            color=(245, 245, 245),
        )


def draw_heads_overlay(frame: Any, heads: list[tuple[int, int, int, int]]) -> None:
    for index, (x, y, width, height) in enumerate(heads, start=1):
        draw_corner_brackets(frame, (x, y, width, height), semantic_color("amber"), label=str(index))


def tail(path: Path, lines: int = 80) -> str:
    if not path.exists():
        return ""
    content = path.read_text(encoding="utf-8", errors="replace").splitlines()
    return "\n".join(content[-lines:])


def status_payload(display_mode: str = "red") -> dict[str, Any]:
    target, _ = latest_target(display_mode, hud_enabled=False)
    port_5600_owners = udp_port_owners(5600)
    camera_dir = active_camera_dir()
    runtime_snapshot = (
        STATE.runtime_steering.snapshot()
        if STATE.runtime_steering is not None and STATE.runtime_steering.running
        else None
    )
    if camera_dir is None and (
        STATE.bridge.running() or process_running(r"gst-launch-1.0 .*port=5600")
    ):
        enable_gazebo_camera_streaming()
    return {
        "vehicle": {
            "mode": STATE.profile.mode,
            "label": STATE.profile.label,
            "steering_supported": STATE.profile.steering_supported,
        },
        "processes": {
            "gazebo": STATE.gazebo.running()
            or process_running(STATE.profile.gazebo_process_pattern),
            "sitl": STATE.sitl.running()
            or process_running(STATE.profile.sitl_process_pattern),
            "bridge": camera_bridge_running(STATE.bridge),
            "target_motion": STATE.target_motion.running()
            or process_running(r"move_gazebo_target.py"),
            "takeoff": STATE.takeoff.running(),
            "steering": STATE.steering.running()
            or (STATE.runtime_steering is not None and STATE.runtime_steering.running),
        },
        "mavlink": mavlink_status(),
        "target": target,
        "camera": {
            "live": camera_dir is not None,
            "udp_5600_owners": port_5600_owners,
            "udp_5600_conflicts": udp_port_conflicts(5600),
        },
        "video": {
            "source": STATE.video_source,
            "input_label": "Gazebo UDP 5600" if STATE.video_source == "udp" else "RTSP",
            "rtsp_url": STATE.rtsp_url,
        },
        "runtime": {
            "simulator": STATE.simulator_mode,
            "active": STATE.runtime_steering is not None and STATE.runtime_steering.running,
            "workers": (
                {}
                if runtime_snapshot is None
                else {
                    name: {
                        "running": health.running,
                        "heartbeat_ns": health.heartbeat_ns,
                        "fault": health.fault,
                    }
                    for name, health in runtime_snapshot.workers.items()
                }
            ),
            "metrics": {} if runtime_snapshot is None else runtime_snapshot.metrics,
        },
        "demand": load_demand_state() or {},
        "selection": selection_payload(),
        "logs": {
            "takeoff": tail(LOG_DIR / "takeoff.log", 40),
            "steering": tail(LOG_DIR / "steering.log", 60),
            "system": "\n".join(STATE.messages[-12:]),
        },
    }


class Handler(BaseHTTPRequestHandler):
    server_version = "VultureXPanel/0.1"

    def log_message(self, _format: str, *args: object) -> None:
        return

    def send_json(self, payload: dict[str, Any], status: HTTPStatus = HTTPStatus.OK) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_body(body, "application/json", status)

    def send_body(
        self,
        body: bytes,
        content_type: str,
        status: HTTPStatus = HTTPStatus.OK,
    ) -> None:
        try:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        except (TimeoutError, BrokenPipeError, ConnectionResetError):
            return

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/":
            body = HTML.encode("utf-8")
            self.send_body(body, "text/html; charset=utf-8")
            return
        if parsed.path == "/api/status":
            query = parse_qs(parsed.query)
            display_mode = query.get("mode", ["red"])[0]
            self.send_json(status_payload(display_mode))
            return
        if parsed.path == "/api/tracking_tuning":
            self.send_json({"ok": True, **current_tuning_payload()})
            return
        if parsed.path == "/api/frame.jpg":
            query = parse_qs(parsed.query)
            display_mode = query.get("mode", ["red"])[0]
            hud_mode = normalize_hud_mode(query.get("hud_mode", ["operational"])[0])
            hud_enabled = hud_enabled_from_query(query.get("hud", ["1"])[0])
            _, image = latest_target(
                display_mode,
                hud_mode=hud_mode,
                hud_enabled=hud_enabled,
            )
            if image is None:
                self.send_response(HTTPStatus.NO_CONTENT)
                self.end_headers()
                return
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(image)))
            try:
                self.end_headers()
                self.wfile.write(image)
            except (TimeoutError, BrokenPipeError, ConnectionResetError):
                return
            return
        self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        query = parse_qs(parsed.query)
        with STATE.lock:
            if parsed.path == "/api/start_gazebo":
                message = STATE.start_gazebo()
            elif parsed.path == "/api/start_sitl":
                message = STATE.start_sitl()
            elif parsed.path == "/api/connect_mavlink":
                mavlink_endpoint = query.get("mavlink", [STATE.mavlink_endpoint])[0]
                message = STATE.connect_mavlink(mavlink_endpoint)
            elif parsed.path == "/api/start_bridge":
                video_source = query.get("video_source", [STATE.video_source])[0]
                rtsp_url = query.get("rtsp_url", [STATE.rtsp_url])[0]
                message = STATE.start_bridge(video_source, rtsp_url)
            elif parsed.path == "/api/start_steering":
                mavlink_endpoint = query.get("mavlink", [STATE.mavlink_endpoint])[0]
                forward_mps = float(query.get("forward_mps", ["3.0"])[0])
                rate_hz = float(query.get("rate_hz", ["30"])[0])
                max_down_mps = float(query.get("max_down_mps", ["3.0"])[0])
                vertical_gain = float(query.get("vertical_gain", ["52"])[0])
                plane_centering_gain = float(query.get("plane_centering_gain", ["1.55"])[0])
                plane_near_centering_gain = float(
                    query.get("plane_near_centering_gain", ["2.65"])[0]
                )
                plane_damping_gain = float(query.get("plane_damping_gain", ["0.14"])[0])
                plane_near_damping_gain = float(
                    query.get("plane_near_damping_gain", ["0.30"])[0]
                )
                plane_far_control_scale = float(
                    query.get("plane_far_control_scale", ["0.72"])[0]
                )
                plane_airspeed_mps = float(query.get("plane_airspeed_mps", ["20.0"])[0])
                max_plane_roll_deg = float(query.get("max_plane_roll_deg", ["35"])[0])
                max_plane_pitch_deg = float(query.get("max_plane_pitch_deg", ["40"])[0])
                plane_pitch_gain_scale = float(query.get("plane_pitch_gain_scale", ["1.20"])[0])
                plane_pitch_near_gain_scale = float(
                    query.get("plane_pitch_near_gain_scale", ["1.60"])[0]
                )
                plane_pitch_below_center_boost = float(
                    query.get("plane_pitch_below_center_boost", ["0.25"])[0]
                )
                plane_pitch_filter_alpha = float(
                    query.get("plane_pitch_filter_alpha", ["0.45"])[0]
                )
                plane_max_pitch_step_deg = float(
                    query.get("plane_max_pitch_step_deg", ["2.0"])[0]
                )
                plane_max_roll_step_deg = float(
                    query.get("plane_max_roll_step_deg", ["6.0"])[0]
                )
                plane_loss_hold_s = float(query.get("plane_loss_hold_s", ["1.5"])[0])
                tracking_mode = query.get("tracking_mode", ["red"])[0]
                surface_test = query.get("surface_test", ["0"])[0] in {"1", "true", "True"}
                message = STATE.start_steering(
                    forward_mps,
                    rate_hz,
                    max_down_mps,
                    vertical_gain,
                    plane_centering_gain,
                    plane_near_centering_gain,
                    plane_damping_gain,
                    plane_near_damping_gain,
                    plane_far_control_scale,
                    max_plane_pitch_deg,
                    plane_pitch_gain_scale,
                    plane_pitch_near_gain_scale,
                    plane_pitch_below_center_boost,
                    plane_pitch_filter_alpha,
                    plane_max_pitch_step_deg,
                    plane_max_roll_step_deg,
                    plane_loss_hold_s,
                    tracking_mode,
                    mavlink_endpoint,
                    surface_test,
                    plane_airspeed_mps,
                    max_plane_roll_deg,
                )
            elif parsed.path == "/api/tracking_tuning":
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    raw_body = self.rfile.read(length) if length > 0 else b"{}"
                    body = json.loads(raw_body.decode("utf-8"))
                    if not isinstance(body, dict):
                        raise ValueError("invalid_payload")
                    payload = write_tracking_tuning(body)
                    self.send_json(payload)
                    return
                except (json.JSONDecodeError, ValueError, OSError) as exc:
                    self.send_json(
                        {"ok": False, "reason": type(exc).__name__},
                        HTTPStatus.BAD_REQUEST,
                    )
                    return
            elif parsed.path == "/api/stop_steering":
                message = STATE.stop_steering()
            elif parsed.path == "/api/start_takeoff":
                mavlink_endpoint = query.get("mavlink", [STATE.mavlink_endpoint])[0]
                message = STATE.start_takeoff(mavlink_endpoint)
            elif parsed.path == "/api/start_target_motion":
                message = STATE.start_target_motion()
            elif parsed.path == "/api/stop_target_motion":
                message = STATE.target_motion.stop()
            elif parsed.path == "/api/stop_all":
                message = "; ".join(STATE.stop_all())
            elif parsed.path == "/api/clear_camera_cache":
                removed = 0
                for directory in [CAMERA_DIR, *Path("/tmp").glob("vulture-x-target-camera-*")]:
                    if not directory.is_dir():
                        continue
                    for image_path in directory.glob("frame-*"):
                        try:
                            image_path.unlink()
                            removed += 1
                        except FileNotFoundError:
                            continue
                message = f"camera cache cleared files={removed}"
            elif parsed.path == "/api/selection":
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    raw_body = self.rfile.read(length).decode("utf-8")
                    raw_selection = json.loads(raw_body) if raw_body else {}
                except (ValueError, json.JSONDecodeError):
                    raw_selection = {}
                message = save_selection(raw_selection)
            elif parsed.path in {"/api/selection/head", "/api/selection/person"}:
                try:
                    x_norm = float(query.get("x", ["nan"])[0])
                    y_norm = float(query.get("y", ["nan"])[0])
                except ValueError:
                    x_norm = float("nan")
                    y_norm = float("nan")
                message = select_head_at(x_norm, y_norm)
            elif parsed.path == "/api/selection/clear":
                with contextlib.suppress(FileNotFoundError):
                    SELECTION_PATH.unlink()
                STATE.selection_tracker.reset()
                message = "custom selection cleared"
            else:
                self.send_error(HTTPStatus.NOT_FOUND)
                return
            STATE.remember(message)
            self.send_json(status_payload())


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    vehicle = parser.add_mutually_exclusive_group()
    vehicle.add_argument(
        "-quad",
        "--quad",
        dest="vehicle",
        action="store_const",
        const="quad",
        help="Start the quadcopter SITL/Gazebo profile. This is the default.",
    )
    vehicle.add_argument(
        "-plane",
        "--plane",
        dest="vehicle",
        action="store_const",
        const="plane",
        help="Start the fixed-wing SITL/Gazebo profile.",
    )
    parser.set_defaults(vehicle=os.environ.get("VULTURE_X_VEHICLE", DEFAULT_VEHICLE_MODE))
    parser.add_argument(
        "--host",
        default="0.0.0.0",
        help="Bind address. Use 0.0.0.0 to allow access from the local network.",
    )
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument(
        "-manual",
        "--manual",
        dest="no_auto_start",
        action="store_true",
        help=(
            "Start only the web UI; do not launch Gazebo, SITL/MAVLink, "
            "camera bridge, or target motion."
        ),
    )
    parser.add_argument(
        "--no-auto-start",
        dest="no_auto_start",
        action="store_true",
        help="Start only the web UI instead of launching Gazebo, SITL, camera, and target motion.",
    )
    parser.add_argument(
        "--no-auto-port",
        action="store_true",
        help="Fail instead of trying the next port when the requested port is busy.",
    )
    return parser.parse_args()


def make_server(host: str, port: int, auto_port: bool) -> tuple[ThreadingHTTPServer, int]:
    last_error: OSError | None = None
    ports = [port] if not auto_port else list(range(port, port + 20))
    for candidate in ports:
        try:
            return ThreadingHTTPServer((host, candidate), Handler), candidate
        except OSError as exc:
            if exc.errno != 98:
                raise
            last_error = exc
    raise RuntimeError(
        f"no free UI port found from {port} to {ports[-1]}"
    ) from last_error


def access_urls(host: str, port: int) -> list[str]:
    if host not in {"0.0.0.0", "::"}:
        return [f"http://{host}:{port}"]

    urls = [f"http://127.0.0.1:{port}"]
    result = subprocess.run(
        ["hostname", "-I"],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        check=False,
    )
    for address in result.stdout.split():
        if address.startswith(("127.", "169.254.")):
            continue
        urls.append(f"http://{address}:{port}")
    return urls


def main() -> int:
    args = parse_args()
    if args.vehicle not in VEHICLE_PROFILES:
        print(
            f"vulture_x_ui_status=failed reason=invalid_vehicle_mode value={args.vehicle}",
            file=sys.stderr,
            flush=True,
        )
        return 2
    STATE.configure(VEHICLE_PROFILES[args.vehicle])
    STATE.configure_runtime_io(simulator=not args.no_auto_start)
    try:
        server, actual_port = make_server(args.host, args.port, not args.no_auto_port)
    except RuntimeError as exc:
        print(f"vulture_x_ui_status=failed reason={exc}", file=sys.stderr, flush=True)
        return 1
    except socket.gaierror as exc:
        print(f"vulture_x_ui_status=failed reason=invalid_host detail={exc}", file=sys.stderr)
        return 1

    urls = access_urls(args.host, actual_port)
    print(f"vulture_x_ui_url={urls[0]}", flush=True)
    print(f"vulture_x_ui_mavlink_status={MAVLINK_STATUS_ENDPOINT}", flush=True)
    for url in urls[1:]:
        print(f"vulture_x_ui_lan_url={url}", flush=True)
    MAVLINK_MONITOR.start()
    if not args.no_browser:
        threading.Timer(0.5, lambda: webbrowser.open(urls[0])).start()
    if not args.no_auto_start:
        for message in STATE.auto_start():
            STATE.remember(message)
            print(f"vulture_x_ui_autostart={message}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        original_sigint = signal.getsignal(signal.SIGINT)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        MAVLINK_MONITOR.stop()
        STATE.stop_all()
        signal.signal(signal.SIGINT, original_sigint)
        server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
