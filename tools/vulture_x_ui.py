#!/usr/bin/env python3
"""Local web control panel for Vulture-X SITL/Gazebo testing."""

from __future__ import annotations

import argparse
import contextlib
import json
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
from mavlink_endpoint import open_mavlink_connection, parse_mavlink_endpoint
from pymavlink import mavutil
from track_camera_target import detect_red_target, newest_image

from vulture_x.vision.tracker import TemplateMatchingTracker

REPO_ROOT = Path(__file__).resolve().parents[1]
LOG_DIR = REPO_ROOT / "logs" / "ui"
CAMERA_DIR = LOG_DIR / "camera_frames"
SELECTION_PATH = LOG_DIR / "custom_selection.json"
DEFAULT_MAVLINK_ENDPOINT = os.environ.get("VULTURE_X_MAVLINK", "udpin:0.0.0.0:14550")
MAVLINK_STATUS_ENDPOINT = os.environ.get("VULTURE_X_UI_MAVLINK", "udpin:0.0.0.0:14552")
DEFAULT_VIDEO_SOURCE = os.environ.get("VULTURE_X_VIDEO_SOURCE", "udp")
DEFAULT_RTSP_URL = os.environ.get("VULTURE_X_RTSP_URL", "")
MAVLINK_STALE_AFTER_S = 3.5
FRESH_FRAME_MAX_AGE_S = 3.0
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
DEFAULT_FIXED_WING_CENTERING_GAIN = 1.15
DEFAULT_FIXED_WING_NEAR_CENTERING_GAIN = 2.15
DEFAULT_FIXED_WING_DAMPING_GAIN = 0.22
DEFAULT_FIXED_WING_NEAR_DAMPING_GAIN = 0.45
DEFAULT_FIXED_WING_PITCH_GAIN_SCALE = 1.10
DEFAULT_FIXED_WING_PITCH_NEAR_GAIN_SCALE = 1.45
DEFAULT_FIXED_WING_FAR_CONTROL_SCALE = 0.55
DEFAULT_FIXED_WING_PITCH_BELOW_BOOST = 0.25
DEFAULT_FIXED_WING_PITCH_FILTER_ALPHA = 0.25
DEFAULT_FIXED_WING_MAX_PITCH_STEP_DEG = 2.0
DEFAULT_FIXED_WING_MAX_ROLL_STEP_DEG = 3.0
DEFAULT_FIXED_WING_LOSS_HOLD_S = 1.5
CAMERA_STREAM_ENABLE_RETRY_S = (0.0, 1.0, 2.5, 5.0, 8.0)


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
      grid-template-columns: 1.35fr 1fr 1fr;
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
          <button class="primary" onclick="steer()">Steer Target</button>
          <button class="secondary" onclick="connectMavlink()">Connect MAVLink</button>
          <button class="secondary" onclick="connectVideo()">Connect Video</button>
        </div>
        <div class="controls">
          <button class="danger" onclick="post('/api/stop_steering')">Stop Steering</button>
          <button onclick="takeoff()">Plane Takeoff</button>
        </div>
      </section>
      <section>
        <h2>Status</h2>
        <div class="status-grid" id="status"></div>
      </section>
      <section>
        <h2>Connections</h2>
        <div class="connection-grid">
          <label>Control MAVLink endpoint
            <input id="mavlink-endpoint" value="udpin:0.0.0.0:14550">
          </label>
          <button class="secondary" onclick="connectMavlink()">Connect</button>
        </div>
        <div class="connection-grid">
          <label>Video source
            <select id="video-source">
              <option value="udp" selected>UDP 5600</option>
              <option value="rtsp">RTSP</option>
            </select>
          </label>
          <button class="secondary" onclick="connectVideo()">Connect</button>
        </div>
        <label>RTSP URL
          <input id="rtsp-url" placeholder="rtsp://127.0.0.1:8554/stream">
        </label>
      </section>
      <section>
        <h2>Environment</h2>
        <div class="environment-controls">
          <button onclick="post('/api/start_gazebo')">Start Gazebo</button>
          <button onclick="post('/api/start_sitl')">Start SITL</button>
          <button onclick="post('/api/start_target_motion')">Move Target</button>
          <button onclick="post('/api/stop_target_motion')">Stop Target</button>
          <button onclick="post('/api/clear_camera_cache')">Clear Camera</button>
          <button class="danger" onclick="post('/api/stop_all')">Stop All</button>
        </div>
      </section>
      <section>
        <h2>Target Steering</h2>
        <div class="mode-row">
          <label>Tracking mode
            <select id="tracking-mode">
              <option value="banner" selected>Banner target</option>
              <option value="custom">Custom selection</option>
            </select>
          </label>
          <label>Selection
            <input id="selection-status" value="none" readonly>
          </label>
        </div>
        <label>Duration seconds
          <input id="duration" type="number" min="3" max="120" value="20">
        </label>
        <div id="quad-fields" class="vehicle-fields">
          <div class="field-title">Quad Guidance</div>
          <div class="inline-fields">
            <label>Forward speed m/s
              <input id="quad-forward-speed" type="number" min="0" max="8" step="0.1" value="3.0">
            </label>
            <label>Command rate Hz
              <input id="quad-command-rate" type="number" min="1" max="20" step="1" value="10">
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
          <div class="field-title">Plane Guidance</div>
          <div class="inline-fields">
          <label>Forward speed m/s
            <input id="plane-forward-speed" type="number" min="0" max="20" step="0.1" value="20.0">
          </label>
          <label>Command rate Hz
            <input id="plane-command-rate" type="number" min="1" max="20" step="1" value="10">
          </label>
          <label>Vertical speed m/s
            <input id="plane-vertical-speed" type="number" min="0" max="10" step="0.1" value="10.0">
          </label>
          <label>Vertical gain
            <input id="plane-vertical-gain" type="number" min="0" max="80" step="1" value="52">
          </label>
          <label>Centering gain
            <input id="plane-centering-gain" type="number" min="0" max="4" step="0.05" value="1.15">
          </label>
          <label>Near centering gain
            <input
              id="plane-near-centering-gain" type="number" min="0" max="4"
              step="0.05" value="2.15"
            >
          </label>
          <label>Damping gain
            <input id="plane-damping-gain" type="number" min="0" max="3" step="0.01" value="0.22">
          </label>
          <label>Near damping gain
            <input
              id="plane-near-damping-gain" type="number" min="0" max="3"
              step="0.01" value="0.45"
            >
          </label>
          <label>Far control scale
            <input
              id="plane-far-control-scale" type="number" min="0.1" max="1"
              step="0.05" value="0.55"
            >
          </label>
          <label>Max plane pitch deg
            <input id="max-plane-pitch" type="number" min="0" max="45" step="1" value="40">
          </label>
          <label>Far pitch gain
            <input id="plane-pitch-gain" type="number" min="0" max="3" step="0.1" value="1.10">
          </label>
          <label>Near pitch gain
            <input id="plane-pitch-near-gain" type="number" min="0" max="5" step="0.1" value="1.45">
          </label>
          <label>Pitch down boost
            <input
              id="plane-pitch-below-boost" type="number" min="0" max="3" step="0.1" value="0.25"
            >
          </label>
          <label>Pitch smoothing
            <input
              id="plane-pitch-filter-alpha" type="number" min="0.05" max="0.8"
              step="0.05" value="0.25"
            >
          </label>
          <label>Max pitch step deg
            <input
              id="plane-max-pitch-step" type="number" min="1" max="8" step="0.5"
              value="2.0"
            >
          </label>
          <label>Max roll step deg
            <input
              id="plane-max-roll-step" type="number" min="1" max="10" step="0.5"
              value="3.0"
            >
          </label>
          <label>Loss hold s
            <input
              id="plane-loss-hold" type="number" min="0" max="5" step="0.1"
              value="1.5"
            >
          </label>
          </div>
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
          <input id="stream-fps" type="number" min="1" max="30" step="1" value="12">
        </label>
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
    function activeNumber(id, fallback) {
      const element = document.getElementById(id);
      return encodeURIComponent(element ? (element.value || fallback) : fallback);
    }
    function mavlinkEndpoint() {
      const element = document.getElementById('mavlink-endpoint');
      const fallback = 'udpin:0.0.0.0:14550';
      return encodeURIComponent(element ? (element.value || fallback) : fallback);
    }
    function videoSource() {
      const element = document.getElementById('video-source');
      return encodeURIComponent(element ? (element.value || 'udp') : 'udp');
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
      const duration = encodeURIComponent(document.getElementById('duration').value || '20');
      const mode = encodeURIComponent(document.getElementById('tracking-mode').value || 'banner');
      const isPlane = currentVehicleMode === 'plane';
      const speed = isPlane
        ? activeNumber('plane-forward-speed', '20.0')
        : activeNumber('quad-forward-speed', '3.0');
      const rate = isPlane
        ? activeNumber('plane-command-rate', '10')
        : activeNumber('quad-command-rate', '10');
      const verticalSpeed = isPlane
        ? activeNumber('plane-vertical-speed', '10.0')
        : activeNumber('quad-vertical-speed', '3.0');
      const verticalGain = isPlane
        ? activeNumber('plane-vertical-gain', '52')
        : activeNumber('quad-vertical-gain', '3.5');
      let path =
        '/api/start_steering?duration=' + duration +
        '&mavlink=' + mavlinkEndpoint() +
        '&forward_mps=' + speed +
        '&rate_hz=' + rate +
        '&max_down_mps=' + verticalSpeed +
        '&vertical_gain=' + verticalGain +
        '&tracking_mode=' + mode;
      if (isPlane) {
        path +=
          '&plane_centering_gain=' + activeNumber('plane-centering-gain', '1.15') +
          '&plane_near_centering_gain=' + activeNumber('plane-near-centering-gain', '2.15') +
          '&plane_damping_gain=' + activeNumber('plane-damping-gain', '0.22') +
          '&plane_near_damping_gain=' + activeNumber('plane-near-damping-gain', '0.45') +
          '&plane_far_control_scale=' + activeNumber('plane-far-control-scale', '0.55') +
          '&max_plane_pitch_deg=' + activeNumber('max-plane-pitch', '40') +
          '&plane_pitch_gain_scale=' + activeNumber('plane-pitch-gain', '1.10') +
          '&plane_pitch_near_gain_scale=' + activeNumber('plane-pitch-near-gain', '1.45') +
          '&plane_pitch_below_center_boost=' + activeNumber('plane-pitch-below-boost', '0.25') +
          '&plane_pitch_filter_alpha=' + activeNumber('plane-pitch-filter-alpha', '0.25') +
          '&plane_max_pitch_step_deg=' + activeNumber('plane-max-pitch-step', '2.0') +
          '&plane_max_roll_step_deg=' + activeNumber('plane-max-roll-step', '3.0') +
          '&plane_loss_hold_s=' + activeNumber('plane-loss-hold', '1.5');
      }
      await post(path);
    }
    async function takeoff() {
      await post('/api/start_takeoff?mavlink=' + mavlinkEndpoint());
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
        row('Gazebo', data.processes.gazebo ? 'running' : 'stopped', data.processes.gazebo),
        row('SITL', data.processes.sitl ? 'running' : 'stopped', data.processes.sitl),
        row('Camera Bridge', data.processes.bridge ? 'running' : 'stopped', data.processes.bridge),
        row('Video Input', data.video.source.toUpperCase(), data.camera.live),
        row(
          'Target Motion',
          data.processes.target_motion ? 'running' : 'stopped',
          data.processes.target_motion
        ),
        row('Live Frames', data.camera.live ? 'live' : 'stale', data.camera.live),
        row('Takeoff', data.processes.takeoff ? 'running' : 'idle', !data.processes.takeoff),
        row(
          'MAVLink',
          data.mavlink.connected ? data.mavlink.mode : 'offline',
          data.mavlink.connected
        ),
        row('Armed', data.mavlink.armed ? 'armed' : 'not armed', data.mavlink.armed),
        row('Target', target, data.target.detected),
        row('Steering', data.processes.steering ? 'running' : 'idle', !data.processes.steering),
      ].join('');
      currentVehicleMode = data.vehicle.mode;
      document.getElementById('quad-fields').hidden = currentVehicleMode !== 'quad';
      document.getElementById('plane-fields').hidden = currentVehicleMode !== 'plane';
      const endpointInput = document.getElementById('mavlink-endpoint');
      if (endpointInput && document.activeElement !== endpointInput) {
        endpointInput.value = data.mavlink.control_endpoint || endpointInput.value;
      }
      const videoSourceInput = document.getElementById('video-source');
      if (videoSourceInput && document.activeElement !== videoSourceInput) {
        videoSourceInput.value = data.video.source || videoSourceInput.value;
      }
      const rtspUrlInput = document.getElementById('rtsp-url');
      if (rtspUrlInput && document.activeElement !== rtspUrlInput) {
        rtspUrlInput.value = data.video.rtsp_url || rtspUrlInput.value;
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
    function refreshFrame() {
      document.getElementById('camera').src = '/api/frame.jpg?t=' + Date.now();
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
      document.getElementById('tracking-mode').value = 'banner';
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
        Math.min(30, Number(document.getElementById('stream-fps').value || '12'))
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
        const response = await fetch('/api/status');
        refresh(await response.json());
      } catch (error) {
        document.getElementById('system-log').textContent = String(error);
      }
    }
    setInterval(poll, 1000);
    installSelectionDrag();
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

    def reset(self) -> None:
        self._tracker = None
        self._selection_mtime_ns = None
        self._last_frame_key = None
        self._last_bbox = None

    def payload(self) -> dict[str, float | bool | str] | None:
        if self._last_bbox is None:
            return None
        frame = newest_image(active_camera_dir() or CAMERA_DIR)
        if frame is None:
            return None
        image = cv2.imread(str(frame))
        if image is None:
            return None
        frame_height, frame_width = image.shape[:2]
        x, y, width, height = self._last_bbox
        return {
            "enabled": True,
            "mode": "custom",
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
            bbox = normalized_selection_to_bbox(selection, frame)
            tracker = create_cv_tracker(self._tracker_name)
            if tracker.init(frame, bbox) is False:
                self.reset()
                return None
            self._tracker = tracker
            self._selection_mtime_ns = selection_mtime_ns
            self._last_frame_key = frame_key
            self._last_bbox = bbox
            return bbox

        detected, raw_bbox = self._tracker.update(frame)
        self._last_frame_key = frame_key
        if not detected:
            self._last_bbox = None
            return None
        self._last_bbox = tuple(round(value) for value in raw_bbox)
        return self._last_bbox


def camera_bridge_command(video_source: str, rtsp_url: str) -> list[str]:
    sink = [
        "!",
        "videoconvert",
        "!",
        "jpegenc",
        "!",
        "multifilesink",
        f"location={CAMERA_DIR}/frame-%06d.jpg",
        "max-files=60",
    ]
    if video_source == "rtsp":
        url = rtsp_url.strip()
        if not url.startswith(("rtsp://", "rtsps://")):
            raise ValueError("rtsp_url_required")
        return [
            "gst-launch-1.0",
            "-q",
            "rtspsrc",
            f"location={url}",
            "latency=100",
            "!",
            "rtph264depay",
            "!",
            "h264parse",
            "!",
            "avdec_h264",
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
                "--timeout-s",
                "20",
            ],
            LOG_DIR / "steering.log",
        )
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

    def remember(self, message: str) -> None:
        self.messages.append(f"{time.strftime('%H:%M:%S')} {message}")
        self.messages = self.messages[-30:]

    def start_steering(
        self,
        duration_s: float,
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
    ) -> str:
        if mavlink_endpoint is None:
            mavlink_endpoint = self.mavlink_endpoint
        try:
            mavlink_endpoint = self.set_mavlink_endpoint(mavlink_endpoint)
        except ValueError as exc:
            return f"steering blocked reason={exc}"
        if not self.profile.steering_supported:
            return "steering blocked reason=fixed_wing_guidance_not_implemented"
        duration_s = max(3.0, min(120.0, duration_s))
        forward_mps = max(0.0, min(self.profile.max_forward_mps, forward_mps))
        rate_hz = max(1.0, min(20.0, rate_hz))
        max_down_mps = max(0.0, min(self.profile.max_vertical_mps, max_down_mps))
        vertical_gain = max(0.0, min(self.profile.max_vertical_gain, vertical_gain))
        plane_centering_gain = max(0.0, min(4.0, plane_centering_gain))
        plane_near_centering_gain = max(0.0, min(4.0, plane_near_centering_gain))
        plane_damping_gain = max(0.0, min(3.0, plane_damping_gain))
        plane_near_damping_gain = max(0.0, min(3.0, plane_near_damping_gain))
        plane_far_control_scale = max(0.1, min(1.0, plane_far_control_scale))
        max_plane_pitch_deg = max(0.0, min(self.profile.max_pitch_deg, max_plane_pitch_deg))
        plane_pitch_gain_scale = max(0.0, min(3.0, plane_pitch_gain_scale))
        plane_pitch_near_gain_scale = max(0.0, min(5.0, plane_pitch_near_gain_scale))
        plane_pitch_below_center_boost = max(0.0, min(3.0, plane_pitch_below_center_boost))
        plane_pitch_filter_alpha = max(0.05, min(0.8, plane_pitch_filter_alpha))
        plane_max_pitch_step_deg = max(1.0, min(8.0, plane_max_pitch_step_deg))
        plane_max_roll_step_deg = max(1.0, min(10.0, plane_max_roll_step_deg))
        plane_loss_hold_s = max(0.0, min(5.0, plane_loss_hold_s))
        if tracking_mode == "orange":
            tracking_mode = "banner"
        if tracking_mode not in {"banner", "custom"}:
            return "steering blocked reason=invalid_tracking_mode"
        if tracking_mode == "custom" and load_selection() is None:
            return "steering blocked reason=missing_custom_selection"
        camera_dir = active_camera_dir()
        if camera_dir is None:
            return "no camera frames available; start camera first"
        self.steering.command = [
            sys.executable,
            "tools/sitl_track_target.py",
            "--enable-guidance",
            "--mavlink",
            mavlink_endpoint,
            "--vehicle",
            self.profile.mode,
            "--camera-dir",
            str(camera_dir),
            "--timeout-s",
            str(duration_s),
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
        ]
        if self.profile.mode == "plane":
            self.steering.command.extend(
                [
                    "--plane-centering-gain",
                    str(plane_centering_gain),
                    "--plane-near-centering-gain",
                    str(plane_near_centering_gain),
                    "--plane-damping-gain",
                    str(plane_damping_gain),
                    "--plane-near-damping-gain",
                    str(plane_near_damping_gain),
                    "--plane-far-control-scale",
                    str(plane_far_control_scale),
                    "--max-plane-pitch-deg",
                    str(max_plane_pitch_deg),
                    "--plane-pitch-gain-scale",
                    str(plane_pitch_gain_scale),
                    "--plane-pitch-near-gain-scale",
                    str(plane_pitch_near_gain_scale),
                    "--plane-pitch-below-center-boost",
                    str(plane_pitch_below_center_boost),
                    "--plane-pitch-filter-alpha",
                    str(plane_pitch_filter_alpha),
                    "--plane-max-pitch-step-deg",
                    str(plane_max_pitch_step_deg),
                    "--plane-max-roll-step-deg",
                    str(plane_max_roll_step_deg),
                    "--plane-loss-hold-s",
                    str(plane_loss_hold_s),
                ]
            )
        if tracking_mode == "custom":
            self.steering.command.extend(["--selection-file", str(SELECTION_PATH)])
        return self.steering.start()

    def start_takeoff(self, mavlink_endpoint: str | None = None) -> str:
        if mavlink_endpoint is None:
            mavlink_endpoint = self.mavlink_endpoint
        try:
            mavlink_endpoint = self.set_mavlink_endpoint(mavlink_endpoint)
        except ValueError as exc:
            return f"takeoff blocked reason={exc}"
        if self.profile.mode != "plane":
            return "takeoff blocked reason=plane_profile_required"
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


def load_selection() -> dict[str, float | bool | str] | None:
    if not SELECTION_PATH.exists():
        return None
    try:
        raw = json.loads(SELECTION_PATH.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    if raw.get("mode") != "custom":
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
        "mode": "custom",
        "x": x,
        "y": y,
        "width": width,
        "height": height,
    }


def selection_payload() -> dict[str, float | bool | str]:
    selection = load_selection()
    if selection is None:
        return {"enabled": False, "mode": "banner", "x": 0.0, "y": 0.0, "width": 0.0, "height": 0.0}
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
    payload = {
        "mode": "custom",
        "x": x,
        "y": y,
        "width": width,
        "height": height,
        "updated_at_unix_s": time.time(),
    }
    SELECTION_PATH.write_text(json.dumps(payload, sort_keys=True), encoding="utf-8")
    STATE.selection_tracker.reset()
    return "custom selection saved"


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
        "/world/vulture_x_test/model/zephyr_with_camera/link/"
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


def mavlink_status() -> dict[str, Any]:
    status = MAVLINK_MONITOR.snapshot()
    status["control_endpoint"] = STATE.mavlink_endpoint
    return status


def latest_target() -> tuple[dict[str, Any], bytes | None]:
    camera_dir = active_camera_dir()
    if camera_dir is None:
        return {"detected": False, "detail": "no camera frame"}, None
    image_path = newest_image(camera_dir)
    if image_path is None:
        return {"detected": False, "detail": "no camera frame"}, None
    frame = cv2.imread(str(image_path))
    if frame is None:
        return {"detected": False, "detail": "frame unreadable"}, None
    selection = load_selection()
    if selection is not None:
        bbox = STATE.selection_tracker.bbox(frame, image_path)
    else:
        bbox = detect_red_target(frame, 25.0)
    target: dict[str, Any] = {"detected": False, "detail": "not detected"}
    if bbox is not None:
        x, y, width, height = bbox
        frame_height, frame_width = frame.shape[:2]
        center_x = (x + width / 2) / frame_width
        center_y = (y + height / 2) / frame_height
        mode = "custom" if selection is not None else "banner"
        target = {
            "detected": True,
            "mode": mode,
            "bbox": [x, y, width, height],
            "center_x": center_x,
            "center_y": center_y,
            "camera_dir": str(camera_dir),
        }
        color = (255, 180, 60) if selection is not None else (0, 255, 255)
        cv2.rectangle(frame, (x, y), (x + width, y + height), color, 2)
        cv2.circle(
            frame,
            (int(center_x * frame_width), int(center_y * frame_height)),
            4,
            color,
            -1,
        )
    elif selection is not None:
        target = {"detected": False, "mode": "custom", "detail": "custom tracking lost"}
        frame_height, frame_width = frame.shape[:2]
        sx = int(float(selection["x"]) * frame_width)
        sy = int(float(selection["y"]) * frame_height)
        sw = int(float(selection["width"]) * frame_width)
        sh = int(float(selection["height"]) * frame_height)
        cv2.rectangle(frame, (sx, sy), (sx + sw, sy + sh), (90, 90, 255), 2)
    draw_center_overlay(frame)
    ok, encoded = cv2.imencode(".jpg", frame)
    return target, encoded.tobytes() if ok else None


def draw_center_overlay(frame: Any) -> None:
    frame_height, frame_width = frame.shape[:2]
    box_width = max(24, round(frame_width * 0.16))
    box_height = max(18, round(frame_height * 0.16))
    center_x = frame_width // 2
    center_y = frame_height // 2
    left = center_x - box_width // 2
    top = center_y - box_height // 2
    right = center_x + box_width // 2
    bottom = center_y + box_height // 2
    color = (255, 255, 255)
    shadow = (0, 0, 0)
    cv2.rectangle(frame, (left, top), (right, bottom), shadow, 3)
    cv2.rectangle(frame, (left, top), (right, bottom), color, 1)
    cv2.line(frame, (center_x - 10, center_y), (center_x + 10, center_y), shadow, 3)
    cv2.line(frame, (center_x, center_y - 10), (center_x, center_y + 10), shadow, 3)
    cv2.line(frame, (center_x - 10, center_y), (center_x + 10, center_y), color, 1)
    cv2.line(frame, (center_x, center_y - 10), (center_x, center_y + 10), color, 1)


def tail(path: Path, lines: int = 80) -> str:
    if not path.exists():
        return ""
    content = path.read_text(encoding="utf-8", errors="replace").splitlines()
    return "\n".join(content[-lines:])


def status_payload() -> dict[str, Any]:
    target, _ = latest_target()
    port_5600_owners = udp_port_owners(5600)
    camera_dir = active_camera_dir()
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
            "steering": STATE.steering.running(),
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
            "rtsp_url": STATE.rtsp_url,
        },
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
            self.send_json(status_payload())
            return
        if parsed.path == "/api/frame.jpg":
            _, image = latest_target()
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
                duration = float(query.get("duration", ["20"])[0])
                forward_mps = float(query.get("forward_mps", ["3.0"])[0])
                rate_hz = float(query.get("rate_hz", ["10"])[0])
                max_down_mps = float(query.get("max_down_mps", ["3.0"])[0])
                vertical_gain = float(query.get("vertical_gain", ["52"])[0])
                plane_centering_gain = float(query.get("plane_centering_gain", ["1.15"])[0])
                plane_near_centering_gain = float(
                    query.get("plane_near_centering_gain", ["2.15"])[0]
                )
                plane_damping_gain = float(query.get("plane_damping_gain", ["0.22"])[0])
                plane_near_damping_gain = float(
                    query.get("plane_near_damping_gain", ["0.45"])[0]
                )
                plane_far_control_scale = float(
                    query.get("plane_far_control_scale", ["0.55"])[0]
                )
                max_plane_pitch_deg = float(query.get("max_plane_pitch_deg", ["40"])[0])
                plane_pitch_gain_scale = float(query.get("plane_pitch_gain_scale", ["1.10"])[0])
                plane_pitch_near_gain_scale = float(
                    query.get("plane_pitch_near_gain_scale", ["1.45"])[0]
                )
                plane_pitch_below_center_boost = float(
                    query.get("plane_pitch_below_center_boost", ["0.25"])[0]
                )
                plane_pitch_filter_alpha = float(
                    query.get("plane_pitch_filter_alpha", ["0.25"])[0]
                )
                plane_max_pitch_step_deg = float(
                    query.get("plane_max_pitch_step_deg", ["2.0"])[0]
                )
                plane_max_roll_step_deg = float(
                    query.get("plane_max_roll_step_deg", ["3.0"])[0]
                )
                plane_loss_hold_s = float(query.get("plane_loss_hold_s", ["1.5"])[0])
                tracking_mode = query.get("tracking_mode", ["banner"])[0]
                message = STATE.start_steering(
                    duration,
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
                )
            elif parsed.path == "/api/stop_steering":
                message = STATE.steering.stop()
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
        "--no-auto-start",
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
