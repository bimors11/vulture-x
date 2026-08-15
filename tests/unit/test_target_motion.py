import importlib.util
import math
from pathlib import Path
from types import ModuleType

import pytest


def load_target_motion_module() -> ModuleType:
    module_path = Path(__file__).parents[2] / "tools" / "move_gazebo_target.py"
    spec = importlib.util.spec_from_file_location("move_gazebo_target", module_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_loiter_pose_orbits_around_center() -> None:
    module = load_target_motion_module()

    x0, y0, z0, yaw0 = module.loiter_pose(
        0.0,
        center_x=12.0,
        center_y=-5.0,
        center_z=6.0,
        radius_y_m=4.0,
        radius_z_m=1.4,
        period_s=5.0,
    )
    x_quarter, y_quarter, z_quarter, _yaw_quarter = module.loiter_pose(
        1.25,
        center_x=12.0,
        center_y=-5.0,
        center_z=6.0,
        radius_y_m=4.0,
        radius_z_m=1.4,
        period_s=5.0,
    )

    assert x0 == pytest.approx(12.0)
    assert y0 == pytest.approx(-1.0)
    assert z0 == pytest.approx(6.0)
    assert yaw0 == pytest.approx(math.pi)
    assert x_quarter == pytest.approx(12.0)
    assert y_quarter == pytest.approx(-5.0)
    assert z_quarter == pytest.approx(7.4)


def test_back_and_forth_pose_reverses_heading() -> None:
    module = load_target_motion_module()

    _x0, y0, _z0, yaw0 = module.back_and_forth_pose(
        0.0,
        center_x=12.0,
        center_y=-5.0,
        center_z=6.0,
        range_y_m=3.5,
        range_z_m=1.2,
        period_s=4.0,
    )
    _x_half, y_half, z_half, yaw_half = module.back_and_forth_pose(
        2.0,
        center_x=12.0,
        center_y=-5.0,
        center_z=6.0,
        range_y_m=3.5,
        range_z_m=1.2,
        period_s=4.0,
    )

    assert y0 == pytest.approx(-5.0)
    assert y_half == pytest.approx(-5.0)
    assert z_half == pytest.approx(7.2)
    assert yaw0 == pytest.approx(math.pi)
    assert yaw_half == pytest.approx(0.0)


def test_euler_to_quaternion_preserves_pitch_component() -> None:
    module = load_target_motion_module()

    qx, qy, qz, qw = module.euler_to_quaternion(0.0, math.radians(8.0), math.radians(-25.0))

    assert qx != pytest.approx(0.0)
    assert qy != pytest.approx(0.0)
    assert qz != pytest.approx(0.0)
    assert qw > 0.0
