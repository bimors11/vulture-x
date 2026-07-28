#!/usr/bin/env python3
"""Local web control panel for Vulture-X SITL/Gazebo testing."""

from __future__ import annotations

import argparse
import json
import os
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
from typing import Any
from urllib.parse import parse_qs, urlparse

import cv2
from pymavlink import mavutil
from track_camera_target import detect_red_target, newest_image

REPO_ROOT = Path(__file__).resolve().parents[1]
LOG_DIR = REPO_ROOT / "logs" / "ui"
CAMERA_DIR = LOG_DIR / "camera_frames"
MAVLINK_ENDPOINT = "udpin:0.0.0.0:14550"
FRESH_FRAME_MAX_AGE_S = 3.0

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
      grid-template-columns: minmax(320px, 420px) minmax(420px, 1fr);
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
    button, input {
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
    input { padding: 0 10px; }
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
    @media (max-width: 900px) {
      main, .two { grid-template-columns: 1fr; }
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
    <div>
      <section>
        <h2>Status</h2>
        <div class="status-grid" id="status"></div>
        <div class="controls">
          <button onclick="post('/api/start_gazebo')">Start Gazebo</button>
          <button onclick="post('/api/start_sitl')">Start SITL</button>
          <button onclick="post('/api/start_bridge')">Start Camera</button>
          <button onclick="post('/api/clear_camera_cache')">Clear Camera</button>
          <button class="danger" onclick="post('/api/stop_all')">Stop All</button>
        </div>
      </section>
      <section style="margin-top:18px">
        <h2>Target Steering</h2>
        <label>Duration seconds
          <input id="duration" type="number" min="3" max="120" value="20">
        </label>
        <div class="controls">
          <button class="primary" onclick="steer()">Steer Toward Target</button>
          <button class="danger" onclick="post('/api/stop_steering')">Stop Steering</button>
        </div>
        <p class="muted">
          Requires SITL already armed and in GUIDED. This panel does not arm or take off.
        </p>
      </section>
    </div>
    <div class="camera-wrap">
      <section>
        <h2>Camera Target View</h2>
        <img id="camera" src="/api/frame.jpg" alt="camera frame">
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
    async function post(path) {
      const response = await fetch(path, {method: 'POST'});
      const data = await response.json();
      refresh(data);
    }
    async function steer() {
      const duration = encodeURIComponent(document.getElementById('duration').value || '20');
      await post('/api/start_steering?duration=' + duration);
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
        row('Gazebo', data.processes.gazebo ? 'running' : 'stopped', data.processes.gazebo),
        row('SITL', data.processes.sitl ? 'running' : 'stopped', data.processes.sitl),
        row('Camera Bridge', data.processes.bridge ? 'running' : 'stopped', data.processes.bridge),
        row('Live Frames', data.camera.live ? 'live' : 'stale', data.camera.live),
        row(
          'MAVLink',
          data.mavlink.connected ? data.mavlink.mode : 'offline',
          data.mavlink.connected
        ),
        row('Armed', data.mavlink.armed ? 'armed' : 'not armed', data.mavlink.armed),
        row('Target', target, data.target.detected),
        row('Steering', data.processes.steering ? 'running' : 'idle', !data.processes.steering),
      ].join('');
      document.getElementById('steer-log').textContent = data.logs.steering;
      const owners = data.camera.udp_5600_owners.join('\n');
      document.getElementById('system-log').textContent =
        data.logs.system + (owners ? '\n\nUDP 5600:\n' + owners : '');
      document.getElementById('camera').src = '/api/frame.jpg?t=' + Date.now();
    }
    async function poll() {
      try {
        const response = await fetch('/api/status');
        refresh(await response.json());
      } catch (error) {
        document.getElementById('system-log').textContent = String(error);
      }
    }
    setInterval(poll, 1000);
    poll();
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
    ):
        self.name = name
        self.command = command
        self.log_path = log_path
        self.env = env
        self.process: subprocess.Popen[str] | None = None

    def running(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def start(self) -> str:
        if self.running():
            return f"{self.name} already running"
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        log_file = self.log_path.open("a", encoding="utf-8")
        started_at = time.strftime("%Y-%m-%d %H:%M:%S")
        log_file.write(f"\n[{started_at}] starting {' '.join(self.command)}\n")
        log_file.flush()
        process_env = os.environ.copy()
        if self.env:
            process_env.update(self.env)
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
        os.killpg(self.process.pid, signal.SIGTERM)
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(self.process.pid, signal.SIGKILL)
            self.process.wait(timeout=5)
        return f"{self.name} stopped"


class AppState:
    def __init__(self) -> None:
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        CAMERA_DIR.mkdir(parents=True, exist_ok=True)
        self.lock = threading.Lock()
        self.gazebo = ManagedProcess("gazebo", ["scripts/run_gazebo.sh"], LOG_DIR / "gazebo.log")
        self.sitl = ManagedProcess(
            "sitl",
            ["scripts/run_sitl.sh"],
            LOG_DIR / "sitl.log",
            {"VULTURE_X_SITL_INTERACTIVE": "0"},
        )
        self.bridge = ManagedProcess(
            "camera bridge",
            [
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
                f"location={CAMERA_DIR}/frame-%06d.jpg",
                "max-files=10",
            ],
            LOG_DIR / "camera_bridge.log",
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

    def remember(self, message: str) -> None:
        self.messages.append(f"{time.strftime('%H:%M:%S')} {message}")
        self.messages = self.messages[-30:]

    def start_steering(self, duration_s: float) -> str:
        duration_s = max(3.0, min(120.0, duration_s))
        camera_dir = active_camera_dir()
        if camera_dir is None:
            return "no camera frames available; start camera first"
        self.steering.command = [
            sys.executable,
            "tools/sitl_track_target.py",
            "--enable-guidance",
            "--camera-dir",
            str(camera_dir),
            "--timeout-s",
            str(duration_s),
        ]
        return self.steering.start()

    def start_gazebo(self) -> str:
        if process_running(r"gz sim .*vulture_x_test"):
            return "gazebo already running"
        return self.gazebo.start()

    def start_sitl(self) -> str:
        if process_running("arducopter --model JSON"):
            return "sitl already running"
        return self.sitl.start()

    def start_bridge(self) -> str:
        if process_running(r"gst-launch-1.0 .*port=5600"):
            return "camera bridge already running"
        enable_gazebo_camera_streaming()
        CAMERA_DIR.mkdir(parents=True, exist_ok=True)
        return self.bridge.start()

    def stop_all(self) -> list[str]:
        return [
            self.steering.stop(),
            self.bridge.stop(),
            self.sitl.stop(),
            self.gazebo.stop(),
        ]


STATE = AppState()


def process_running(pattern: str) -> bool:
    result = subprocess.run(
        ["pgrep", "-f", pattern],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return result.returncode == 0


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


def enable_gazebo_camera_streaming() -> None:
    topic = (
        "/world/vulture_x_test/model/iris_with_gimbal/model/gimbal/link/"
        "pitch_link/sensor/camera/image/enable_streaming"
    )
    subprocess.run(
        ["gz", "topic", "-t", topic, "-m", "gz.msgs.Boolean", "-p", "data: true"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        text=True,
        check=False,
    )


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
    try:
        connection = mavutil.mavlink_connection(
            MAVLINK_ENDPOINT,
            source_system=201,
            source_component=202,
            autoreconnect=False,
        )
        heartbeat = connection.wait_heartbeat(timeout=0.8)
    except Exception as exc:
        return {"connected": False, "armed": False, "mode": "offline", "detail": str(exc)}
    try:
        if heartbeat is None:
            return {"connected": False, "armed": False, "mode": "offline", "detail": "no heartbeat"}
        armed = bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        return {
            "connected": True,
            "armed": armed,
            "mode": mavutil.mode_string_v10(heartbeat),
            "type": int(heartbeat.type),
            "autopilot": int(heartbeat.autopilot),
        }
    finally:
        connection.close()


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
    bbox = detect_red_target(frame, 80.0)
    target: dict[str, Any] = {"detected": False, "detail": "not detected"}
    if bbox is not None:
        x, y, width, height = bbox
        frame_height, frame_width = frame.shape[:2]
        center_x = (x + width / 2) / frame_width
        center_y = (y + height / 2) / frame_height
        target = {
            "detected": True,
            "bbox": [x, y, width, height],
            "center_x": center_x,
            "center_y": center_y,
            "camera_dir": str(camera_dir),
        }
        cv2.rectangle(frame, (x, y), (x + width, y + height), (0, 255, 255), 2)
        cv2.circle(
            frame,
            (int(center_x * frame_width), int(center_y * frame_height)),
            4,
            (0, 255, 255),
            -1,
        )
    ok, encoded = cv2.imencode(".jpg", frame)
    return target, encoded.tobytes() if ok else None


def tail(path: Path, lines: int = 80) -> str:
    if not path.exists():
        return ""
    content = path.read_text(encoding="utf-8", errors="replace").splitlines()
    return "\n".join(content[-lines:])


def status_payload() -> dict[str, Any]:
    target, _ = latest_target()
    port_5600_owners = udp_port_owners(5600)
    return {
        "processes": {
            "gazebo": STATE.gazebo.running() or process_running(r"gz sim .*vulture_x_test"),
            "sitl": STATE.sitl.running() or process_running("arducopter --model JSON"),
            "bridge": STATE.bridge.running() or process_running(r"gst-launch-1.0 .*port=5600"),
            "steering": STATE.steering.running(),
        },
        "mavlink": mavlink_status(),
        "target": target,
        "camera": {
            "live": active_camera_dir() is not None,
            "udp_5600_owners": port_5600_owners,
        },
        "logs": {
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
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/":
            body = HTML.encode("utf-8")
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
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
            self.end_headers()
            self.wfile.write(image)
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
            elif parsed.path == "/api/start_bridge":
                message = STATE.start_bridge()
            elif parsed.path == "/api/start_steering":
                duration = float(query.get("duration", ["20"])[0])
                message = STATE.start_steering(duration)
            elif parsed.path == "/api/stop_steering":
                message = STATE.steering.stop()
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
            else:
                self.send_error(HTTPStatus.NOT_FOUND)
                return
            STATE.remember(message)
            self.send_json(status_payload())


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--host",
        default="0.0.0.0",
        help="Bind address. Use 0.0.0.0 to allow access from the local network.",
    )
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-browser", action="store_true")
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
    for url in urls[1:]:
        print(f"vulture_x_ui_lan_url={url}", flush=True)
    if not args.no_browser:
        threading.Timer(0.5, lambda: webbrowser.open(urls[0])).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        STATE.stop_all()
        server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
