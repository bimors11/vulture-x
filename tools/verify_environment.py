#!/usr/bin/env python3
"""Verify the local Vulture-X SITL/Gazebo development environment."""

from __future__ import annotations

import argparse
import importlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import cv2
from pymavlink import mavutil

DEFAULT_CAMERA_PIPELINE = (
    "udpsrc port=5600 caps=application/x-rtp,media=video,clock-rate=90000,"
    "encoding-name=H264 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink sync=false"
)


def default_ardupilot_dir() -> Path:
    ardu_sitl = Path.home() / "ArduSITL" / "ardupilot"
    if ardu_sitl.exists():
        return ardu_sitl
    return Path.home() / "ardupilot"


@dataclass(frozen=True)
class CheckResult:
    name: str
    ok: bool
    detail: str
    version: str | None = None


def run_command(command: list[str], cwd: Path | None = None) -> tuple[bool, str]:
    try:
        completed = subprocess.run(
            command,
            cwd=cwd,
            check=False,
            capture_output=True,
            text=True,
            timeout=10,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        return False, str(exc)
    output = (completed.stdout or completed.stderr).strip()
    return completed.returncode == 0, output


def check_python(_args: argparse.Namespace) -> CheckResult:
    version = ".".join(str(part) for part in sys.version_info[:3])
    ok = sys.version_info >= (3, 11)
    return CheckResult("python", ok, sys.executable, version)


def check_import(module_name: str, version_attr: str = "__version__") -> CheckResult:
    try:
        module = importlib.import_module(module_name)
    except ImportError as exc:
        return CheckResult(module_name, False, str(exc))
    version = getattr(module, version_attr, None)
    return CheckResult(module_name, True, "import ok", str(version) if version else None)


def check_opencv(_args: argparse.Namespace) -> CheckResult:
    return CheckResult("opencv", True, "import ok", cv2.__version__)


def check_pymavlink(_args: argparse.Namespace) -> CheckResult:
    try:
        importlib.import_module("pymavlink")
    except ImportError as exc:
        return CheckResult("pymavlink", False, str(exc))
    return CheckResult("pymavlink", True, "import ok", getattr(mavutil, "__version__", None))


def check_gazebo(_args: argparse.Namespace) -> CheckResult:
    gz = shutil.which("gz")
    if gz:
        ok, output = run_command([gz, "sim", "--versions"])
        if ok:
            version = output.splitlines()[0] if output else None
            return CheckResult("gazebo", True, "gz sim available", version)
        ok, output = run_command([gz, "sim", "-h"])
        if ok:
            return CheckResult("gazebo", True, "gz sim available", None)

    classic = shutil.which("gazebo")
    if classic:
        ok, output = run_command([classic, "--version"])
        version = output.splitlines()[0] if output else None
        detail = "Gazebo Classic found; current ArduPilot Gazebo Sim plugin still needs gz sim"
        return CheckResult("gazebo", False, detail, version)

    return CheckResult("gazebo", False, "neither gz sim nor gazebo was found")


def check_ardupilot(args: argparse.Namespace) -> CheckResult:
    sim_vehicle = args.ardupilot_dir / "Tools" / "autotest" / "sim_vehicle.py"
    copter_binary = args.ardupilot_dir / "build" / "sitl" / "bin" / "arducopter"
    if not sim_vehicle.exists():
        return CheckResult("ardupilot_sitl", False, f"missing {sim_vehicle}")
    if not copter_binary.exists():
        return CheckResult("ardupilot_sitl", False, f"missing built binary {copter_binary}")
    ok, commit = run_command(["git", "rev-parse", "--short", "HEAD"], cwd=args.ardupilot_dir)
    version = commit if ok else None
    return CheckResult("ardupilot_sitl", True, str(copter_binary), version)


def check_mavlink(args: argparse.Namespace) -> CheckResult:
    try:
        connection = mavutil.mavlink_connection(
            args.mavlink,
            source_system=191,
            source_component=191,
            autoreconnect=False,
        )
        heartbeat = connection.wait_heartbeat(timeout=args.heartbeat_timeout_s)
        connection.close()
    except Exception as exc:  # pymavlink raises broad transport/parser exceptions.
        return CheckResult("mavlink_heartbeat", False, str(exc))

    if heartbeat is None:
        return CheckResult(
            "mavlink_heartbeat",
            False,
            f"no heartbeat within {args.heartbeat_timeout_s}s",
        )
    vehicle_type = mavutil.mavlink.enums["MAV_TYPE"].get(heartbeat.type)
    autopilot = mavutil.mavlink.enums["MAV_AUTOPILOT"].get(heartbeat.autopilot)
    detail = (
        f"type={vehicle_type.name if vehicle_type else heartbeat.type} "
        f"autopilot={autopilot.name if autopilot else heartbeat.autopilot}"
    )
    return CheckResult("mavlink_heartbeat", True, detail)


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


def check_camera(args: argparse.Namespace) -> CheckResult:
    deadline = time.monotonic() + args.camera_timeout_s
    if args.camera_dir is not None:
        while time.monotonic() < deadline:
            image_path = newest_image(args.camera_dir)
            frame = cv2.imread(str(image_path)) if image_path else None
            if frame is not None:
                height, width = frame.shape[:2]
                detail = f"{image_path} width={width} height={height}"
                return CheckResult("camera_frame", True, detail)
            time.sleep(0.1)
        return CheckResult("camera_frame", False, f"no image files in {args.camera_dir}")

    capture = cv2.VideoCapture(args.camera_pipeline, cv2.CAP_GSTREAMER)
    if not capture.isOpened():
        capture.release()
        capture = cv2.VideoCapture(args.camera_pipeline)
    if not capture.isOpened():
        bridge_result = check_camera_with_gstreamer_bridge(args.camera_timeout_s)
        if bridge_result.ok:
            return bridge_result
        return CheckResult(
            "camera_frame",
            False,
            f"OpenCV could not open camera pipeline; {bridge_result.detail}",
        )

    try:
        while time.monotonic() < deadline:
            ok, frame = capture.read()
            if ok and frame is not None:
                height, width = frame.shape[:2]
                return CheckResult("camera_frame", True, f"width={width} height={height}")
            time.sleep(0.05)
    finally:
        capture.release()
    return CheckResult("camera_frame", False, f"no frame within {args.camera_timeout_s}s")


def check_camera_with_gstreamer_bridge(timeout_s: float) -> CheckResult:
    if shutil.which("gst-launch-1.0") is None:
        return CheckResult("camera_frame", False, "gst-launch-1.0 not found")

    camera_dir = Path(tempfile.mkdtemp(prefix="vulture-x-camera-"))
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
    try:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            image_path = newest_image(camera_dir)
            frame = cv2.imread(str(image_path)) if image_path else None
            if frame is not None:
                height, width = frame.shape[:2]
                return CheckResult(
                    "camera_frame",
                    True,
                    f"{image_path} width={width} height={height}",
                )
            time.sleep(0.1)
        return CheckResult("camera_frame", False, f"no bridged frame in {camera_dir}")
    finally:
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ardupilot-dir", type=Path, default=default_ardupilot_dir())
    parser.add_argument(
        "--mavlink",
        default=os.environ.get("VULTURE_X_MAVLINK", "udpin:0.0.0.0:14550"),
    )
    parser.add_argument("--heartbeat-timeout-s", type=float, default=15.0)
    parser.add_argument(
        "--camera-pipeline",
        default=os.environ.get("VULTURE_X_CAMERA_PIPELINE", DEFAULT_CAMERA_PIPELINE),
    )
    parser.add_argument("--camera-dir", type=Path, default=None)
    parser.add_argument("--camera-timeout-s", type=float, default=10.0)
    parser.add_argument(
        "--checks",
        default="python,opencv,pymavlink,gazebo,ardupilot,mavlink,camera",
        help="Comma-separated checks to run.",
    )
    parser.add_argument("--json", action="store_true", help="Emit JSON Lines.")
    return parser.parse_args()


def print_result(result: CheckResult, json_output: bool) -> None:
    if json_output:
        print(json.dumps(result.__dict__, sort_keys=True), flush=True)
        return
    status = "PASS" if result.ok else "FAIL"
    version = f" version={result.version}" if result.version else ""
    print(f"[{status}] {result.name}{version}: {result.detail}", flush=True)


def main() -> int:
    args = parse_args()
    checks: dict[str, Callable[[argparse.Namespace], CheckResult]] = {
        "python": check_python,
        "opencv": check_opencv,
        "pymavlink": check_pymavlink,
        "gazebo": check_gazebo,
        "ardupilot": check_ardupilot,
        "mavlink": check_mavlink,
        "camera": check_camera,
        "pytest": lambda _args: check_import("pytest"),
        "ruff": lambda _args: check_import("ruff"),
        "mypy": lambda _args: check_import("mypy.version", "__version__"),
    }
    requested = [name.strip() for name in args.checks.split(",") if name.strip()]
    results: list[CheckResult] = []

    for name in requested:
        check = checks.get(name)
        if check is None:
            results.append(CheckResult(name, False, "unknown check"))
            continue
        result = check(args)
        results.append(result)
        print_result(result, args.json)

    return 0 if all(result.ok for result in results) else 1


if __name__ == "__main__":
    sys.exit(main())
