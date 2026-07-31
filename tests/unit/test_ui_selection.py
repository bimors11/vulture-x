import importlib.util
import sys
from pathlib import Path
from types import ModuleType


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
    assert (
        state.start_steering(
            duration_s=20,
            forward_mps=20,
            rate_hz=10,
            max_down_mps=10,
            vertical_gain=40,
            min_relative_alt_m=15,
            max_plane_pitch_deg=20,
            tracking_mode="orange",
        )
        == "steering started"
    )
    assert "--vehicle" in state.steering.command
    assert "plane" in state.steering.command
    assert state.steering.command[state.steering.command.index("--forward-mps") + 1] == "20.0"
    assert state.steering.command[state.steering.command.index("--max-down-mps") + 1] == "10.0"
    assert state.steering.command[state.steering.command.index("--vertical-gain") + 1] == "40"
    assert "--plane-vertical-lookahead-s" not in state.steering.command
    assert (
        state.steering.command[state.steering.command.index("--min-relative-alt-m") + 1]
        == "15.0"
    )
    assert (
        state.steering.command[state.steering.command.index("--max-plane-pitch-deg") + 1]
        == "20.0"
    )
    assert "--max-plane-roll-deg" not in state.steering.command


def test_ui_mavlink_status_uses_dedicated_sitl_stream() -> None:
    module = load_ui_module()
    run_sitl = Path("scripts/run_sitl.sh").read_text(encoding="utf-8")

    assert module.MAVLINK_STATUS_ENDPOINT == "udpin:0.0.0.0:14552"
    assert "--out=udp:127.0.0.1:14552" in run_sitl
