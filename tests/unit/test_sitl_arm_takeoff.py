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
