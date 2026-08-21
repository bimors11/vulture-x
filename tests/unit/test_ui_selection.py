import importlib.util
import os
import socket
import sys
import time
from pathlib import Path
from types import ModuleType, SimpleNamespace

import cv2
import numpy as np

from vulture_x.runtime.state import VehicleSnapshot


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


def test_head_selection_round_trip(tmp_path: Path) -> None:
    module = load_ui_module()
    module.SELECTION_PATH = tmp_path / "selection.json"

    message = module.save_selection(
        {"mode": "head", "x": 0.2, "y": 0.3, "width": 0.2, "height": 0.5}
    )

    assert message == "head selection saved"
    assert module.selection_payload()["mode"] == "head"


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


def test_manual_argument_disables_auto_start(monkeypatch) -> None:
    module = load_ui_module()
    monkeypatch.setattr(sys, "argv", ["vulture_x_ui.py", "-manual"])

    args = module.parse_args()

    assert args.no_auto_start is True


def test_runtime_io_defaults_to_simulator_when_auto_starting() -> None:
    module = load_ui_module()
    state = module.AppState(module.VEHICLE_PROFILES["plane"])

    state.configure_runtime_io(simulator=True)

    assert state.mavlink_endpoint == "udpin:0.0.0.0:14550"
    assert state.video_source == "udp"
    assert state.rtsp_url == ""


def test_status_labels_simulator_camera_as_gazebo_udp(tmp_path: Path, monkeypatch) -> None:
    module = load_ui_module()
    module.LOG_DIR = tmp_path
    module.CAMERA_DIR = tmp_path / "camera_frames"
    state = module.AppState(module.VEHICLE_PROFILES["plane"])
    state.configure_runtime_io(simulator=True)
    monkeypatch.setattr(module, "STATE", state)

    payload = module.status_payload()

    assert payload["video"]["source"] == "udp"
    assert payload["video"]["input_label"] == "Gazebo UDP 5600"
    assert payload["runtime"]["simulator"] is True


def test_status_payload_uses_requested_display_mode(monkeypatch) -> None:
    module = load_ui_module()
    seen: list[str] = []

    def fake_latest_target(
        display_mode: str = "red",
        **_kwargs: object,
    ) -> tuple[dict[str, object], None]:
        seen.append(display_mode)
        return {"detected": False, "mode": display_mode}, None

    monkeypatch.setattr(module, "latest_target", fake_latest_target)

    payload = module.status_payload("banner")

    assert seen == ["banner"]
    assert payload["target"]["mode"] == "banner"


def test_simulator_controls_exist_but_start_hidden() -> None:
    module = load_ui_module()

    assert 'id="sim-actions"' in module.HTML
    assert "Start Gazebo" in module.HTML
    assert "Start SITL" in module.HTML
    assert "Plane Takeoff" in module.HTML
    assert "Start Target" in module.HTML


def test_runtime_io_keeps_hardware_defaults_for_manual() -> None:
    module = load_ui_module()
    state = module.AppState(module.VEHICLE_PROFILES["plane"])

    state.configure_runtime_io(simulator=False)

    assert state.mavlink_endpoint == "udpcl:192.168.144.12:19856"
    assert state.video_source == "rtsp"
    assert state.rtsp_url == "rtsp://192.168.144.25:8554/main.264"


def test_tracking_tuning_write_is_atomic_and_clamped(tmp_path: Path) -> None:
    module = load_ui_module()
    module.TRACKING_TUNING_PATH = tmp_path / "tracking_tuning.json"

    payload = module.write_tracking_tuning(
        {
            "plane_centering_gain": 9.0,
            "plane_min_throttle": 0.7,
            "plane_max_throttle": 0.2,
            "plane_proximity_far_size": 0.4,
            "plane_proximity_near_size": 0.1,
        }
    )

    assert payload["ok"] is True
    assert payload["revision"] == 1
    assert payload["values"]["plane_centering_gain"] == 4.0
    assert payload["values"]["plane_max_throttle"] == 0.7
    assert payload["values"]["plane_proximity_near_size"] > 0.4
    assert module.TRACKING_TUNING_PATH.exists()
    assert not (tmp_path / "tracking_tuning.json.tmp").exists()


def test_app_state_configures_plane_commands(tmp_path: Path, monkeypatch) -> None:
    module = load_ui_module()
    monkeypatch.setenv("VULTURE_X_LEGACY_STEERING", "1")
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
            plane_pitch_filter_alpha=0.45,
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
    assert state.steering.command[state.steering.command.index("--source-system") + 1] == "255"
    assert "--timeout-s" not in state.steering.command
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
        == "0.45"
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
    assert (
        state.steering.command[state.steering.command.index("--max-plane-roll-deg") + 1]
        == "35.0"
    )


def test_app_state_plane_start_uses_runtime_by_default(tmp_path: Path, monkeypatch) -> None:
    module = load_ui_module()
    module.LOG_DIR = tmp_path
    module.CAMERA_DIR = tmp_path / "camera_frames"
    module.CAMERA_DIR.mkdir(parents=True)
    (module.CAMERA_DIR / "frame-000001.jpg").write_bytes(b"not-a-real-test-image")
    subprocess_starts: list[str] = []
    runtime_starts: list[object] = []
    monkeypatch.setattr(
        module.ManagedProcess,
        "start",
        lambda self: subprocess_starts.append(self.name) or "steering started",
    )

    class FakeRuntimeSteeringSession:
        def __init__(self, **kwargs: object) -> None:
            self.kwargs = kwargs
            self.running = False

        def start(self) -> str:
            self.running = True
            runtime_starts.append(self)
            return "runtime steering started"

        def stop(self, *, request_auto: bool) -> str:
            self.running = False
            self.request_auto = request_auto
            return "runtime steering stopped"

    monkeypatch.setattr(module, "RuntimeSteeringSession", FakeRuntimeSteeringSession)
    state = module.AppState(module.VEHICLE_PROFILES["plane"])

    assert (
        state.start_steering(
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
            plane_pitch_filter_alpha=0.45,
            plane_max_pitch_step_deg=2.0,
            plane_max_roll_step_deg=3.0,
            plane_loss_hold_s=1.5,
            tracking_mode="banner",
            mavlink_endpoint="udpcl:192.168.144.12:19856",
        )
        == "runtime steering started"
    )

    assert runtime_starts
    assert subprocess_starts == []
    assert state.runtime_steering is runtime_starts[0]
    assert runtime_starts[0].kwargs["mavlink_endpoint"] == "udpcl:192.168.144.12:19856"


def test_app_state_stop_shuts_runtime_without_legacy_subprocess(
    tmp_path: Path,
    monkeypatch,
) -> None:
    module = load_ui_module()
    module.LOG_DIR = tmp_path
    state = module.AppState(module.VEHICLE_PROFILES["plane"])
    state.configure_runtime_io(simulator=True)
    stopped: list[bool] = []

    class FakeRuntime:
        running = True

        def stop(self, *, request_auto: bool) -> str:
            stopped.append(request_auto)
            self.running = False
            return "runtime steering stopped"

    state.runtime_steering = FakeRuntime()

    assert state.stop_steering() == "runtime steering stopped"
    assert stopped == [True]
    assert state.runtime_steering is None


def test_runtime_session_pilot_takeover_releases_without_auto(monkeypatch) -> None:
    module = load_ui_module()
    auto_calls: list[object] = []

    class FakeConnection:
        def set_mode(self, mode: object) -> None:
            auto_calls.append(mode)

    session = object.__new__(module.RuntimeSteeringSession)
    session.abort_reason = None
    session.connection = FakeConnection()
    session.control_authority_active = True
    session.plane_tracking_started = True
    session.runtime = None
    session.running = True
    session.last_command_ns = None
    session.tracker_engine_name = "TEMPLATE"
    session.response_model = module.stt.plane_response_model_from_params(
        module.stt.PlaneTrackingTuning(
            revision=0,
            vertical_gain=52.0,
            plane_centering_gain=1.55,
            plane_near_centering_gain=2.65,
            plane_roll_gain_scale=1.75,
            plane_pitch_gain_scale=1.20,
            plane_pitch_near_gain_scale=1.60,
            plane_error_deadband=0.015,
            plane_lead_s=0.0,
            plane_damping_gain=0.14,
            plane_near_damping_gain=0.30,
            plane_pitch_filter_alpha=0.45,
            plane_max_pitch_step_deg=2.0,
            plane_max_roll_step_deg=6.0,
            max_plane_roll_deg=35.0,
            max_plane_pitch_deg=40.0,
            plane_near_pitch_down_limit_deg=40.0,
            plane_far_control_scale=0.72,
            plane_near_control_scale=1.0,
            plane_camera_hfov_deg=70.0,
            plane_proximity_far_size=0.025,
            plane_proximity_near_size=0.16,
            plane_airspeed_mps=20.0,
            plane_throttle=0.55,
            plane_throttle_airspeed_gain=0.04,
            plane_min_throttle=0.25,
            plane_max_throttle=0.80,
            plane_near_throttle_reduction=0.0,
            plane_pitch_below_center_boost=0.25,
            plane_loss_hold_s=1.5,
            min_tracking_alt_m=15.0,
            airspeed_low_persistence_s=2.0,
        ),
        {},
    )
    session.tracking_mode = "banner"
    session.rate_hz = 30.0
    session.tuning = module.stt.plane_tuning_from_args(
        type(
            "Args",
            (),
            {
                "vertical_gain": 52.0,
                "plane_centering_gain": 1.55,
                "plane_near_centering_gain": 2.65,
                "plane_roll_gain_scale": 1.75,
                "plane_pitch_gain_scale": 1.20,
                "plane_pitch_near_gain_scale": 1.60,
                "plane_error_deadband": 0.015,
                "plane_lead_s": 0.0,
                "plane_damping_gain": 0.14,
                "plane_near_damping_gain": 0.30,
                "plane_pitch_filter_alpha": 0.45,
                "plane_max_pitch_step_deg": 2.0,
                "plane_max_roll_step_deg": 6.0,
                "max_plane_roll_deg": 35.0,
                "max_plane_pitch_deg": 40.0,
                "plane_near_pitch_down_limit_deg": 40.0,
                "plane_far_control_scale": 0.72,
                "plane_near_control_scale": 1.0,
                "plane_camera_hfov_deg": 70.0,
                "plane_proximity_far_size": 0.025,
                "plane_proximity_near_size": 0.16,
                "plane_airspeed_mps": 20.0,
                "plane_throttle": 0.55,
                "plane_throttle_airspeed_gain": 0.04,
                "plane_min_throttle": 0.25,
                "plane_max_throttle": 0.80,
                "plane_near_throttle_reduction": 0.0,
                "plane_pitch_below_center_boost": 0.25,
                "plane_loss_hold_s": 1.5,
                "min_tracking_alt_m": 15.0,
                "airspeed_low_persistence_s": 2.0,
            },
        )()
    )
    released: list[object] = []
    monkeypatch.setattr(module.stt, "release_rc_override", released.append)
    vehicle = VehicleSnapshot(
        timestamp_ns=10_000_000,
        heartbeat_timestamp_ns=10_000_000,
        mode="MANUAL",
        armed=True,
        relative_altitude_m=50.0,
        airspeed_mps=20.0,
        connected=True,
    )
    inputs = module.ControlInputs(
        now_ns=10_000_000,
        frame=None,
        tracking=None,
        vehicle=vehicle,
        frame_age_ms=0.0,
        tracking_result_age_ms=None,
        heartbeat_age_ms=0.0,
    )

    session._control_step(inputs)

    assert session.abort_reason == "pilot_mode_change"
    assert session.control_authority_active is False
    assert released == [session.connection]
    assert auto_calls == []


def test_app_state_configures_plane_surface_test_command(
    tmp_path: Path,
    monkeypatch,
) -> None:
    module = load_ui_module()
    monkeypatch.setenv("VULTURE_X_LEGACY_STEERING", "1")
    module.LOG_DIR = tmp_path
    module.CAMERA_DIR = tmp_path / "camera_frames"
    module.SELECTION_PATH = tmp_path / "custom_selection.json"
    module.CAMERA_DIR.mkdir(parents=True)
    (module.CAMERA_DIR / "frame-000001.jpg").write_bytes(b"not-a-real-test-image")
    module.SELECTION_PATH.write_text(
        '{"mode":"custom","x":0.25,"y":0.25,"width":0.1,"height":0.1}',
        encoding="utf-8",
    )
    monkeypatch.setattr(module.ManagedProcess, "start", lambda self: "steering started")
    state = module.AppState(module.VEHICLE_PROFILES["plane"])

    assert (
        state.start_steering(
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
            plane_pitch_filter_alpha=0.45,
            plane_max_pitch_step_deg=2.0,
            plane_max_roll_step_deg=3.0,
            plane_loss_hold_s=1.5,
            tracking_mode="custom",
            mavlink_endpoint="udpout:192.168.144.12:19856",
            surface_test=True,
        )
        == "steering started"
    )
    assert "--surface-test" in state.steering.command
    assert state.steering.command[state.steering.command.index("--mavlink") + 1] == (
        "udpout:192.168.144.12:19856"
    )
    assert state.steering.command[state.steering.command.index("--selection-file") + 1] == str(
        module.SELECTION_PATH
    )
    assert (
        state.steering.command[state.steering.command.index("--plane-param-cache-file") + 1]
        == str(module.PLANE_PARAM_CACHE_PATH)
    )
    assert state.steering.command[state.steering.command.index("--surface-test-throttle") + 1] == (
        "0.0"
    )


def test_simulator_plane_ground_test_uses_zero_throttle(
    tmp_path: Path,
    monkeypatch,
) -> None:
    module = load_ui_module()
    monkeypatch.setenv("VULTURE_X_LEGACY_STEERING", "1")
    module.LOG_DIR = tmp_path
    module.CAMERA_DIR = tmp_path / "camera_frames"
    module.SELECTION_PATH = tmp_path / "custom_selection.json"
    module.CAMERA_DIR.mkdir(parents=True)
    (module.CAMERA_DIR / "frame-000001.jpg").write_bytes(b"not-a-real-test-image")
    module.SELECTION_PATH.write_text(
        '{"mode":"custom","x":0.25,"y":0.25,"width":0.1,"height":0.1}',
        encoding="utf-8",
    )
    monkeypatch.setattr(module.ManagedProcess, "start", lambda self: "steering started")
    state = module.AppState(module.VEHICLE_PROFILES["plane"])
    state.configure_runtime_io(simulator=True)

    assert (
        state.start_steering(
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
            plane_pitch_filter_alpha=0.45,
            plane_max_pitch_step_deg=2.0,
            plane_max_roll_step_deg=3.0,
            plane_loss_hold_s=1.5,
            tracking_mode="custom",
            mavlink_endpoint="udpin:0.0.0.0:14550",
            surface_test=True,
        )
        == "steering started"
    )

    assert "--surface-test-throttle" in state.steering.command
    assert (
        state.steering.command[state.steering.command.index("--surface-test-throttle") + 1]
        == "0.0"
    )
    assert state.steering.command[state.steering.command.index("--plane-throttle") + 1] == "0.8"
    assert state.steering.command[state.steering.command.index("--plane-min-throttle") + 1] == (
        "0.8"
    )
    assert state.steering.command[state.steering.command.index("--plane-max-throttle") + 1] == (
        "0.8"
    )


def test_simulator_plane_tracking_uses_eighty_percent_throttle(
    tmp_path: Path,
    monkeypatch,
) -> None:
    module = load_ui_module()
    monkeypatch.setenv("VULTURE_X_LEGACY_STEERING", "1")
    module.LOG_DIR = tmp_path
    module.CAMERA_DIR = tmp_path / "camera_frames"
    module.TRACKING_TUNING_PATH = tmp_path / "tracking_tuning.json"
    module.CAMERA_DIR.mkdir(parents=True)
    (module.CAMERA_DIR / "frame-000001.jpg").write_bytes(b"not-a-real-test-image")
    monkeypatch.setattr(module.ManagedProcess, "start", lambda self: "steering started")
    state = module.AppState(module.VEHICLE_PROFILES["plane"])
    state.configure_runtime_io(simulator=True)

    assert (
        state.start_steering(
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
            plane_pitch_filter_alpha=0.45,
            plane_max_pitch_step_deg=2.0,
            plane_max_roll_step_deg=3.0,
            plane_loss_hold_s=1.5,
            tracking_mode="banner",
            mavlink_endpoint="udpin:0.0.0.0:14550",
        )
        == "steering started"
    )

    assert "--surface-test" not in state.steering.command
    assert state.steering.command[state.steering.command.index("--plane-throttle") + 1] == "0.8"
    assert state.steering.command[state.steering.command.index("--plane-min-throttle") + 1] == (
        "0.8"
    )
    assert state.steering.command[state.steering.command.index("--plane-max-throttle") + 1] == (
        "0.8"
    )


def test_app_state_surface_test_tracks_red_without_manual_selection(
    tmp_path: Path,
    monkeypatch,
) -> None:
    module = load_ui_module()
    monkeypatch.setenv("VULTURE_X_LEGACY_STEERING", "1")
    module.LOG_DIR = tmp_path
    module.CAMERA_DIR = tmp_path / "camera_frames"
    module.SELECTION_PATH = tmp_path / "custom_selection.json"
    module.CAMERA_DIR.mkdir(parents=True)
    (module.CAMERA_DIR / "frame-000001.jpg").write_bytes(b"not-a-real-test-image")
    monkeypatch.setattr(module.ManagedProcess, "start", lambda self: "steering started")
    state = module.AppState(module.VEHICLE_PROFILES["plane"])

    assert (
        state.start_steering(
            forward_mps=20,
            rate_hz=15,
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
            plane_pitch_filter_alpha=0.45,
            plane_max_pitch_step_deg=2.0,
            plane_max_roll_step_deg=3.0,
            plane_loss_hold_s=1.5,
            tracking_mode="red",
            mavlink_endpoint="udpcl:192.168.144.12:19856",
            surface_test=True,
        )
        == "steering started"
    )
    assert state.steering.command[state.steering.command.index("--tracking-mode") + 1] == "red"
    assert state.steering.command[state.steering.command.index("--mavlink") + 1] == (
        "udpcl:192.168.144.12:19856"
    )
    assert state.steering.command[state.steering.command.index("--rate-hz") + 1] == "15"
    assert "--surface-test" in state.steering.command
    assert "--selection-file" not in state.steering.command


def test_app_state_surface_test_tracks_selected_head(
    tmp_path: Path,
    monkeypatch,
) -> None:
    module = load_ui_module()
    monkeypatch.setenv("VULTURE_X_LEGACY_STEERING", "1")
    module.LOG_DIR = tmp_path
    module.CAMERA_DIR = tmp_path / "camera_frames"
    module.SELECTION_PATH = tmp_path / "custom_selection.json"
    module.CAMERA_DIR.mkdir(parents=True)
    (module.CAMERA_DIR / "frame-000001.jpg").write_bytes(b"not-a-real-test-image")
    module.SELECTION_PATH.write_text(
        '{"mode":"head","x":0.25,"y":0.25,"width":0.1,"height":0.3}',
        encoding="utf-8",
    )
    monkeypatch.setattr(module.ManagedProcess, "start", lambda self: "steering started")
    state = module.AppState(module.VEHICLE_PROFILES["plane"])

    assert (
        state.start_steering(
            forward_mps=20,
            rate_hz=30,
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
            plane_pitch_filter_alpha=0.45,
            plane_max_pitch_step_deg=2.0,
            plane_max_roll_step_deg=3.0,
            plane_loss_hold_s=1.5,
            tracking_mode="head",
            mavlink_endpoint="udpcl:192.168.144.12:19856",
            surface_test=True,
        )
        == "steering started"
    )
    assert state.steering.command[state.steering.command.index("--tracking-mode") + 1] == "head"
    assert state.steering.command[state.steering.command.index("--selection-file") + 1] == str(
        module.SELECTION_PATH
    )


def test_latest_target_uses_ui_tracker_for_custom_selection(
    tmp_path: Path,
    monkeypatch,
) -> None:
    module = load_ui_module()
    module.LOG_DIR = tmp_path
    module.CAMERA_DIR = tmp_path / "camera_frames"
    module.SELECTION_PATH = tmp_path / "custom_selection.json"
    module.CAMERA_DIR.mkdir(parents=True)
    frame_path = module.CAMERA_DIR / "frame-000001.jpg"
    frame = np.zeros((120, 200, 3), dtype=np.uint8)
    cv2.rectangle(frame, (64, 38), (104, 86), (0, 0, 230), -1)
    cv2.imwrite(str(frame_path), frame)
    old_mtime = time.time() - 1.0
    os.utime(frame_path, (old_mtime, old_mtime))
    module.SELECTION_PATH.write_text(
        '{"mode":"custom","x":0.1,"y":0.1,"width":0.8,"height":0.7}',
        encoding="utf-8",
    )
    state = module.AppState(module.VEHICLE_PROFILES["plane"])
    monkeypatch.setattr(module, "STATE", state)

    target, image = module.latest_target()

    assert image is not None
    assert target["detected"] is True
    x, y, width, height = target["bbox"]
    assert x > 45
    assert y > 25
    assert x + width < 125
    assert y + height < 100


def test_latest_target_prefers_tracking_demand_bbox_for_custom_selection(
    tmp_path: Path,
    monkeypatch,
) -> None:
    module = load_ui_module()
    module.LOG_DIR = tmp_path
    module.CAMERA_DIR = tmp_path / "camera_frames"
    module.SELECTION_PATH = tmp_path / "custom_selection.json"
    module.DEMAND_STATE_PATH = tmp_path / "tracking_demand.json"
    module.CAMERA_DIR.mkdir(parents=True)
    frame_path = module.CAMERA_DIR / "frame-000001.jpg"
    frame = np.zeros((120, 200, 3), dtype=np.uint8)
    cv2.imwrite(str(frame_path), frame)
    old_mtime = time.time() - 1.0
    os.utime(frame_path, (old_mtime, old_mtime))
    module.SELECTION_PATH.write_text(
        '{"mode":"custom","x":0.1,"y":0.1,"width":0.8,"height":0.7}',
        encoding="utf-8",
    )
    module.DEMAND_STATE_PATH.write_text(
        (
            '{"mode":"custom","detected":true,"bbox":[70,40,36,42],'
            f'"updated_unix_s":{time.time()}}}'
        ),
        encoding="utf-8",
    )
    state = module.AppState(module.VEHICLE_PROFILES["plane"])
    monkeypatch.setattr(module, "STATE", state)
    monkeypatch.setattr(state.steering, "running", lambda: True)
    monkeypatch.setattr(
        state.selection_tracker,
        "bbox",
        lambda _frame, _image_path: (_ for _ in ()).throw(AssertionError("fallback used")),
    )

    target, image = module.latest_target()

    assert image is not None
    assert target["bbox"] == [70, 40, 36, 42]


def test_latest_target_uses_banner_detector_even_with_saved_selection(
    tmp_path: Path,
    monkeypatch,
) -> None:
    module = load_ui_module()
    module.LOG_DIR = tmp_path
    module.CAMERA_DIR = tmp_path / "camera_frames"
    module.SELECTION_PATH = tmp_path / "custom_selection.json"
    module.CAMERA_DIR.mkdir(parents=True)
    frame_path = module.CAMERA_DIR / "frame-000001.jpg"
    frame = np.zeros((120, 200, 3), dtype=np.uint8)
    cv2.imwrite(str(frame_path), frame)
    old_mtime = time.time() - 1.0
    os.utime(frame_path, (old_mtime, old_mtime))
    module.SELECTION_PATH.write_text(
        '{"mode":"custom","x":0.1,"y":0.1,"width":0.8,"height":0.7}',
        encoding="utf-8",
    )
    state = module.AppState(module.VEHICLE_PROFILES["plane"])
    monkeypatch.setattr(module, "STATE", state)
    monkeypatch.setattr(module, "detect_banner_target", lambda _frame, _min_area: (20, 30, 40, 50))
    monkeypatch.setattr(
        module,
        "detect_colored_target",
        lambda _frame, _min_area: (_ for _ in ()).throw(AssertionError("red detector used")),
    )
    monkeypatch.setattr(
        state.selection_tracker,
        "bbox",
        lambda _frame, _image_path: (_ for _ in ()).throw(AssertionError("selection used")),
    )

    target, image = module.latest_target("banner")

    assert image is not None
    assert target["mode"] == "banner"
    assert target["bbox"] == [20, 30, 40, 50]


def test_select_head_at_saves_clicked_head_bbox(
    tmp_path: Path,
    monkeypatch,
) -> None:
    module = load_ui_module()
    module.LOG_DIR = tmp_path
    module.CAMERA_DIR = tmp_path / "camera_frames"
    module.SELECTION_PATH = tmp_path / "custom_selection.json"
    module.CAMERA_DIR.mkdir(parents=True)
    frame_path = module.CAMERA_DIR / "frame-000001.jpg"
    frame = np.zeros((120, 200, 3), dtype=np.uint8)
    cv2.imwrite(str(frame_path), frame)
    old_mtime = time.time() - 1.0
    os.utime(frame_path, (old_mtime, old_mtime))
    monkeypatch.setattr(
        module,
        "detect_heads_yunet",
        lambda _frame, **_kwargs: [(10, 20, 30, 80), (100, 15, 40, 90)],
    )

    message = module.select_head_at(0.60, 0.50)

    assert message == "head selection saved heads_detected=2"
    payload = module.selection_payload()
    assert payload["mode"] == "head"
    assert payload["x"] == 0.5
    assert payload["width"] == 0.2


def test_latest_target_draws_head_count_without_selection(
    tmp_path: Path,
    monkeypatch,
) -> None:
    module = load_ui_module()
    module.LOG_DIR = tmp_path
    module.CAMERA_DIR = tmp_path / "camera_frames"
    module.SELECTION_PATH = tmp_path / "custom_selection.json"
    module.CAMERA_DIR.mkdir(parents=True)
    frame_path = module.CAMERA_DIR / "frame-000001.jpg"
    frame = np.zeros((120, 200, 3), dtype=np.uint8)
    cv2.imwrite(str(frame_path), frame)
    old_mtime = time.time() - 1.0
    os.utime(frame_path, (old_mtime, old_mtime))
    monkeypatch.setattr(
        module,
        "detect_heads_yunet",
        lambda _frame, **_kwargs: [(10, 20, 30, 80), (100, 15, 40, 90)],
    )

    target, image = module.latest_target("head")

    assert image is not None
    assert target["mode"] == "head"
    assert target["head_count"] == 2


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
            plane_pitch_filter_alpha=0.45,
            plane_max_pitch_step_deg=2.0,
            plane_max_roll_step_deg=3.0,
            plane_loss_hold_s=1.5,
            tracking_mode="banner",
            mavlink_endpoint="udpin:0.0.0.0:14550",
        )
        == "steering started"
    )
    assert state.steering.command[state.steering.command.index("--vehicle") + 1] == "quad"
    assert "--timeout-s" not in state.steering.command
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
    state.configure_runtime_io(simulator=True)

    assert state.start_takeoff("udpin:0.0.0.0:14550") == "takeoff started"
    assert state.takeoff.command == [
        sys.executable,
        "tools/sitl_arm_takeoff.py",
        "--mavlink",
        "udpin:0.0.0.0:14550",
        "--vehicle",
        "plane",
        "--altitude-m",
        "50",
    ]
    assert state.takeoff.env == {"VULTURE_X_ALLOW_SITL_ARM": "1"}


class FakeUiMavConnection:
    def __init__(self) -> None:
        self.modes: list[int] = []
        self.closed = False
        self.mav = self

    def heartbeat_send(self, *args: object) -> None:
        del args

    def wait_heartbeat(self, timeout: float) -> object:
        del timeout
        return object()

    def mode_mapping(self) -> dict[str, int]:
        return {"AUTO": 10}

    def set_mode(self, mode: int) -> None:
        self.modes.append(mode)

    def close(self) -> None:
        self.closed = True


def test_stop_steering_returns_sim_plane_to_auto(tmp_path: Path, monkeypatch) -> None:
    module = load_ui_module()
    module.LOG_DIR = tmp_path
    module.CAMERA_DIR = tmp_path / "camera_frames"
    connection = FakeUiMavConnection()
    monkeypatch.setattr(module.ManagedProcess, "stop", lambda self: "steering stopped")
    monkeypatch.setattr(module, "open_mavlink_connection", lambda *_args, **_kwargs: connection)
    state = module.AppState(module.VEHICLE_PROFILES["plane"])
    state.configure_runtime_io(simulator=True)

    assert state.stop_steering() == "steering stopped; auto commanded"
    assert connection.modes == [10]
    assert connection.closed is True


def test_manual_mode_blocks_plane_takeoff(tmp_path: Path) -> None:
    module = load_ui_module()
    module.LOG_DIR = tmp_path
    module.CAMERA_DIR = tmp_path / "camera_frames"
    state = module.AppState(module.VEHICLE_PROFILES["plane"])
    state.configure_runtime_io(simulator=False)

    assert state.start_takeoff("serial:/dev/ttyUSB1:57600") == (
        "takeoff blocked reason=simulator_only"
    )


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
        "uridecodebin",
        "uri=rtsp://127.0.0.1:8554/cam",
        "source::latency=50",
    ]
    assert "source::drop-on-latency=true" in state.bridge.command
    assert "source::do-retransmission=false" in state.bridge.command
    assert "source::protocols=tcp" in state.bridge.command
    assert "max-rate=30" in state.bridge.command
    assert "rtph264depay" not in state.bridge.command
    assert "avdec_h264" not in state.bridge.command
    assert f"location={module.CAMERA_DIR}/frame-%06d.jpg" in state.bridge.command


def test_mavlink_connect_probe_updates_control_endpoint(tmp_path: Path, monkeypatch) -> None:
    module = load_ui_module()
    module.LOG_DIR = tmp_path
    module.CAMERA_DIR = tmp_path / "camera_frames"
    opened: list[tuple[str, int, int]] = []

    class FakeConnection:
        closed = False
        mav = SimpleNamespace(heartbeat_send=lambda *_args: None)

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


def test_mavlink_endpoint_parser_accepts_mission_planner_udpcl() -> None:
    module = load_ui_module()

    parsed = module.parse_mavlink_endpoint("udpcl:192.168.144.12:19856")

    assert parsed.device == "udpout:192.168.144.12:19856"


def test_open_mavlink_connection_udpcl_connects_remote_udp_peer() -> None:
    module = load_ui_module()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server:
        server.bind(("127.0.0.1", 0))
        port = server.getsockname()[1]

        connection = module.open_mavlink_connection(
            f"udpcl:127.0.0.1:{port}",
            source_system=255,
            source_component=0,
        )
        try:
            assert connection.port.getpeername() == ("127.0.0.1", port)
        finally:
            connection.close()
