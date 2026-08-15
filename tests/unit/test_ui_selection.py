import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import numpy as np


def load_ui_module() -> ModuleType:
    tools_dir = Path("tools").resolve()
    if str(tools_dir) not in sys.path:
        sys.path.insert(0, str(tools_dir))
    spec = importlib.util.spec_from_file_location(
        "vulture_x_ui_test_module",
        tools_dir / "vulture_x_ui.py",
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_custom_selection_round_trip(tmp_path: Path) -> None:
    module = load_ui_module()
    module.SELECTION_PATH = tmp_path / "selection.json"

    message = module.save_selection({"x": 0.1, "y": 0.2, "width": 0.3, "height": 0.4})

    assert message == "custom selection saved"
    assert module.selection_payload() == {
        "enabled": True,
        "mode": "custom",
        "tracking": "initializing",
        "x": 0.1,
        "y": 0.2,
        "width": 0.3,
        "height": 0.4,
    }


def test_custom_selection_rejects_bad_geometry(tmp_path: Path) -> None:
    module = load_ui_module()
    module.SELECTION_PATH = tmp_path / "selection.json"

    assert (
        module.save_selection({"x": 0.95, "y": 0.2, "width": 0.1, "height": 0.4})
        == "selection rejected reason=selection_outside_frame"
    )
    assert (
        module.save_selection({"x": 0.1, "y": 0.2, "width": 0.01, "height": 0.4})
        == "selection rejected reason=selection_too_small"
    )
    assert module.selection_payload()["enabled"] is False


def test_vehicle_argument_defaults_to_quad(monkeypatch) -> None:
    module = load_ui_module()
    monkeypatch.setattr(sys, "argv", ["vulture_x_ui.py"])

    args = module.parse_args()

    assert args.vehicle == "quad"


def test_vehicle_argument_accepts_plane(monkeypatch) -> None:
    module = load_ui_module()
    monkeypatch.setattr(sys, "argv", ["vulture_x_ui.py", "-plane"])

    args = module.parse_args()

    assert args.vehicle == "plane"


def test_app_state_configures_plane_commands(tmp_path: Path, monkeypatch) -> None:
    module = load_ui_module()
    module.LOG_DIR = tmp_path
    module.CAMERA_DIR = tmp_path / "camera_frames"
    module.CAMERA_DIR.mkdir(parents=True)
    (module.CAMERA_DIR / "frame-000001.jpg").write_bytes(b"not-a-real-test-image")
    monkeypatch.setattr(module.ManagedProcess, "start", lambda self: "steering started")
    state = module.AppState()

    state.configure(module.VEHICLE_PROFILES["plane"])

    assert state.gazebo.command == ["scripts/run_gazebo.sh", "-plane"]
    assert state.sitl.command == ["scripts/run_sitl.sh", "-plane"]
    assert state.target_motion.command == [
        sys.executable,
        "tools/move_gazebo_target.py",
        "--world",
        "vulture_x_plane",
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
    ]
    assert (
        state.start_steering(
            duration_s=20,
            forward_mps=20,
            rate_hz=10,
            max_down_mps=10,
            vertical_gain=52,
            plane_centering_gain=1.15,
            plane_near_centering_gain=2.15,
            plane_damping_gain=0.22,
            plane_near_damping_gain=0.45,
            plane_far_control_scale=0.55,
            max_plane_pitch_deg=40,
            plane_pitch_gain_scale=1.10,
            plane_pitch_near_gain_scale=1.45,
            plane_pitch_below_center_boost=0.25,
            plane_pitch_filter_alpha=0.25,
            plane_max_pitch_step_deg=2.0,
            plane_max_roll_step_deg=3.0,
            plane_loss_hold_s=1.5,
            tracking_mode="banner",
            mavlink_endpoint="serial:/dev/ttyUSB0:57600",
        )
        == "steering started"
    )
    assert state.steering.command[state.steering.command.index("--mavlink") + 1] == (
        "serial:/dev/ttyUSB0:57600"
    )
    assert "--vehicle" in state.steering.command
    assert "plane" in state.steering.command
    assert state.steering.command[state.steering.command.index("--forward-mps") + 1] == "20.0"
    assert state.steering.command[state.steering.command.index("--max-down-mps") + 1] == "10.0"
    assert state.steering.command[state.steering.command.index("--vertical-gain") + 1] == "52"
    assert (
        state.steering.command[state.steering.command.index("--plane-centering-gain") + 1]
        == "1.15"
    )
    assert (
        state.steering.command[state.steering.command.index("--plane-near-centering-gain") + 1]
        == "2.15"
    )
    assert (
        state.steering.command[state.steering.command.index("--plane-damping-gain") + 1]
        == "0.22"
    )
    assert (
        state.steering.command[state.steering.command.index("--plane-near-damping-gain") + 1]
        == "0.45"
    )
    assert (
        state.steering.command[state.steering.command.index("--plane-far-control-scale") + 1]
        == "0.55"
    )
    assert "--plane-vertical-lookahead-s" not in state.steering.command
    assert "--min-relative-alt-m" not in state.steering.command
    assert (
        state.steering.command[state.steering.command.index("--max-plane-pitch-deg") + 1]
        == "40.0"
    )
    assert (
        state.steering.command[state.steering.command.index("--plane-pitch-gain-scale") + 1]
        == "1.1"
    )
    assert (
        state.steering.command[state.steering.command.index("--plane-pitch-near-gain-scale") + 1]
        == "1.45"
    )
    assert (
        state.steering.command[
            state.steering.command.index("--plane-pitch-below-center-boost") + 1
        ]
        == "0.25"
    )
    assert (
        state.steering.command[state.steering.command.index("--plane-pitch-filter-alpha") + 1]
        == "0.25"
    )
    assert (
        state.steering.command[state.steering.command.index("--plane-max-pitch-step-deg") + 1]
        == "2.0"
    )
    assert (
        state.steering.command[state.steering.command.index("--plane-max-roll-step-deg") + 1]
        == "3.0"
    )
    assert (
        state.steering.command[state.steering.command.index("--plane-loss-hold-s") + 1]
        == "1.5"
    )
    assert "--max-plane-roll-deg" not in state.steering.command


def test_app_state_configures_quad_commands_without_plane_parameters(
    tmp_path: Path,
    monkeypatch,
) -> None:
    module = load_ui_module()
    module.LOG_DIR = tmp_path
    module.CAMERA_DIR = tmp_path / "camera_frames"
    module.CAMERA_DIR.mkdir(parents=True)
    (module.CAMERA_DIR / "frame-000001.jpg").write_bytes(b"not-a-real-test-image")
    monkeypatch.setattr(module.ManagedProcess, "start", lambda self: "steering started")
    state = module.AppState()

    state.configure(module.VEHICLE_PROFILES["quad"])

    assert (
        state.start_steering(
            duration_s=20,
            forward_mps=3.0,
            rate_hz=10,
            max_down_mps=3.0,
            vertical_gain=3.5,
            plane_centering_gain=1.15,
            plane_near_centering_gain=2.15,
            plane_damping_gain=0.22,
            plane_near_damping_gain=0.45,
            plane_far_control_scale=0.55,
            max_plane_pitch_deg=40,
            plane_pitch_gain_scale=1.10,
            plane_pitch_near_gain_scale=1.45,
            plane_pitch_below_center_boost=0.25,
            plane_pitch_filter_alpha=0.25,
            plane_max_pitch_step_deg=2.0,
            plane_max_roll_step_deg=3.0,
            plane_loss_hold_s=1.5,
            tracking_mode="banner",
            mavlink_endpoint="udpin:0.0.0.0:14550",
        )
        == "steering started"
    )
    assert state.steering.command[state.steering.command.index("--vehicle") + 1] == "quad"
    assert state.steering.command[state.steering.command.index("--forward-mps") + 1] == "3.0"
    assert state.steering.command[state.steering.command.index("--vertical-gain") + 1] == "3.5"
    assert "--plane-centering-gain" not in state.steering.command
    assert "--plane-near-centering-gain" not in state.steering.command
    assert "--plane-damping-gain" not in state.steering.command
    assert "--plane-near-damping-gain" not in state.steering.command
    assert "--plane-far-control-scale" not in state.steering.command
    assert "--max-plane-pitch-deg" not in state.steering.command
    assert "--plane-pitch-gain-scale" not in state.steering.command
    assert "--plane-max-roll-step-deg" not in state.steering.command
    assert "--plane-loss-hold-s" not in state.steering.command


def test_plane_takeoff_button_command_is_guarded(tmp_path: Path, monkeypatch) -> None:
    module = load_ui_module()
    module.LOG_DIR = tmp_path
    module.CAMERA_DIR = tmp_path / "camera_frames"
    monkeypatch.setattr(module.ManagedProcess, "start", lambda self: "takeoff started")
    state = module.AppState(module.VEHICLE_PROFILES["plane"])

    assert state.start_takeoff("serial:/dev/ttyUSB1:57600") == "takeoff started"
    assert state.takeoff.command == [
        sys.executable,
        "tools/sitl_arm_takeoff.py",
        "--mavlink",
        "serial:/dev/ttyUSB1:57600",
        "--vehicle",
        "plane",
        "--altitude-m",
        "50",
    ]
    assert state.takeoff.env == {"VULTURE_X_ALLOW_SITL_ARM": "1"}


def test_quad_profile_blocks_plane_takeoff(tmp_path: Path) -> None:
    module = load_ui_module()
    module.LOG_DIR = tmp_path
    module.CAMERA_DIR = tmp_path / "camera_frames"
    state = module.AppState(module.VEHICLE_PROFILES["quad"])

    assert state.start_takeoff() == "takeoff blocked reason=plane_profile_required"


def test_camera_bridge_accepts_rtsp_input(tmp_path: Path, monkeypatch) -> None:
    module = load_ui_module()
    module.LOG_DIR = tmp_path
    module.CAMERA_DIR = tmp_path / "camera_frames"
    monkeypatch.setattr(module.ManagedProcess, "start", lambda self: "camera bridge started")
    monkeypatch.setattr(module, "camera_bridge_running", lambda _bridge: False)
    state = module.AppState(module.VEHICLE_PROFILES["plane"])

    assert state.start_bridge("rtsp", "rtsp://127.0.0.1:8554/cam") == "camera bridge started"
    assert state.video_source == "rtsp"
    assert state.rtsp_url == "rtsp://127.0.0.1:8554/cam"
    assert state.bridge.command[:5] == [
        "gst-launch-1.0",
        "-q",
        "rtspsrc",
        "location=rtsp://127.0.0.1:8554/cam",
        "latency=100",
    ]
    assert f"location={module.CAMERA_DIR}/frame-%06d.jpg" in state.bridge.command


def test_mavlink_connect_probe_updates_control_endpoint(tmp_path: Path, monkeypatch) -> None:
    module = load_ui_module()
    module.LOG_DIR = tmp_path
    module.CAMERA_DIR = tmp_path / "camera_frames"
    opened: list[tuple[str, int, int]] = []

    class FakeConnection:
        closed = False

        def wait_heartbeat(self, timeout: float) -> object:
            assert timeout == 3.0
            return SimpleNamespace()

        def close(self) -> None:
            self.closed = True

    fake = FakeConnection()

    def fake_open(endpoint: str, *, source_system: int, source_component: int, autoreconnect: bool):
        assert autoreconnect is False
        opened.append((endpoint, source_system, source_component))
        return fake

    monkeypatch.setattr(module, "open_mavlink_connection", fake_open)
    monkeypatch.setattr(module.mavutil, "mode_string_v10", lambda _heartbeat: "FBWA")
    state = module.AppState(module.VEHICLE_PROFILES["plane"])

    message = state.connect_mavlink("serial:/dev/ttyUSB0:57600")

    assert message == "mavlink connected endpoint=serial:/dev/ttyUSB0:57600 mode=FBWA"
    assert state.mavlink_endpoint == "serial:/dev/ttyUSB0:57600"
    assert opened == [("serial:/dev/ttyUSB0:57600", 201, 203)]
    assert fake.closed is True


def test_center_overlay_draws_without_source_selection_state(tmp_path: Path) -> None:
    module = load_ui_module()
    module.SELECTION_PATH = tmp_path / "selection.json"
    frame = np.zeros((120, 160, 3), dtype=np.uint8)

    module.draw_center_overlay(frame)

    assert frame[60, 80].any()
    assert module.selection_payload()["enabled"] is False


def test_ui_mavlink_status_uses_dedicated_sitl_stream() -> None:
    module = load_ui_module()
    run_sitl = Path("scripts/run_sitl.sh").read_text(encoding="utf-8")

    assert module.MAVLINK_STATUS_ENDPOINT == "udpin:0.0.0.0:14552"
    assert "--out=udp:127.0.0.1:14552" in run_sitl


def test_mavlink_endpoint_parser_accepts_serial() -> None:
    module = load_ui_module()

    parsed = module.parse_mavlink_endpoint("serial:/dev/ttyUSB0:57600")

    assert parsed.device == "/dev/ttyUSB0"
    assert parsed.baud == 57600
