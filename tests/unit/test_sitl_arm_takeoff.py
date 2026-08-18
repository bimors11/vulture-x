import importlib.util
import sys
from pathlib import Path
from types import ModuleType


def load_takeoff_module() -> ModuleType:
    tools_dir = Path("tools").resolve()
    if str(tools_dir) not in sys.path:
        sys.path.insert(0, str(tools_dir))
    spec = importlib.util.spec_from_file_location(
        "sitl_arm_takeoff_test_module",
        tools_dir / "sitl_arm_takeoff.py",
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_source_system_defaults_to_ardupilot_gcs_id(monkeypatch) -> None:
    module = load_takeoff_module()
    monkeypatch.delenv("VULTURE_X_MAVLINK_SOURCE_SYSTEM", raising=False)
    monkeypatch.setattr(sys, "argv", ["sitl_arm_takeoff.py"])

    args = module.parse_args()

    assert args.source_system == 255


def test_source_system_can_follow_custom_mav_gcs_sysid(monkeypatch) -> None:
    module = load_takeoff_module()
    monkeypatch.setenv("VULTURE_X_MAVLINK_SOURCE_SYSTEM", "191")
    monkeypatch.setattr(sys, "argv", ["sitl_arm_takeoff.py"])

    args = module.parse_args()

    assert args.source_system == 191


class FakeMav:
    def __init__(self) -> None:
        self.rc_override_calls: list[tuple[object, ...]] = []

    def rc_channels_override_send(self, *args: object) -> None:
        self.rc_override_calls.append(args)


class FakeConnection:
    def __init__(self) -> None:
        self.mav = FakeMav()
        self.target_system = 1
        self.target_component = 1
        self.modes: list[int] = []

    def mode_mapping(self) -> dict[str, int]:
        return {"FBWA": 5, "AUTO": 10}

    def set_mode(self, mode: int) -> None:
        self.modes.append(mode)


def test_plane_takeoff_switches_to_auto_and_refreshes_throttle(monkeypatch) -> None:
    module = load_takeoff_module()
    connection = FakeConnection()

    monkeypatch.setattr(module, "wait_mode", lambda *_args: True)
    monkeypatch.setattr(module, "arm_vehicle", lambda *_args: True)
    monkeypatch.setattr(module, "wait_groundspeed", lambda *_args: True)
    monkeypatch.setattr(module, "wait_relative_altitude", lambda *_args: True)

    result = module.plane_takeoff(connection, altitude_m=50.0, timeout_s=1.0)

    assert result == 0
    assert connection.modes == [5, 10]
    assert len(connection.mav.rc_override_calls) == 2
    assert connection.mav.rc_override_calls[-1][4] == 1800
