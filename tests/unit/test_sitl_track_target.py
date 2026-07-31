import importlib.util
import sys
from pathlib import Path
from types import ModuleType

from pymavlink import mavutil


def load_tracking_module() -> ModuleType:
    tools_dir = Path("tools").resolve()
    if str(tools_dir) not in sys.path:
        sys.path.insert(0, str(tools_dir))
    spec = importlib.util.spec_from_file_location(
        "sitl_track_target_test_module",
        tools_dir / "sitl_track_target.py",
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeMav:
    def __init__(self) -> None:
        self.command_int_calls: list[tuple[object, ...]] = []
        self.local_ned_calls: list[tuple[object, ...]] = []
        self.attitude_target_calls: list[tuple[object, ...]] = []

    def command_int_send(self, *args: object) -> None:
        self.command_int_calls.append(args)

    def set_position_target_local_ned_send(self, *args: object) -> None:
        self.local_ned_calls.append(args)

    def set_attitude_target_send(self, *args: object) -> None:
        self.attitude_target_calls.append(args)


class FakeConnection:
    def __init__(self) -> None:
        self.mav = FakeMav()


def test_plane_speed_uses_guided_airspeed_command() -> None:
    module = load_tracking_module()
    connection = FakeConnection()

    module.send_plane_speed(connection, 1, 1, 25.0)

    call = connection.mav.command_int_calls[-1]
    assert call[3] == mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_SPEED
    assert call[6] == mavutil.mavlink.SPEED_TYPE_AIRSPEED
    assert call[7] == 25.0


def test_plane_altitude_uses_guided_altitude_slew_command() -> None:
    module = load_tracking_module()
    connection = FakeConnection()

    module.send_plane_altitude(connection, 1, 1, 60.0, 10.0)

    call = connection.mav.command_int_calls[-1]
    assert call[3] == mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_ALTITUDE
    assert call[8] == 10.0
    assert call[12] == 60.0


def test_plane_target_altitude_uses_lookahead_and_floor() -> None:
    module = load_tracking_module()

    assert module.plane_target_altitude(
        relative_altitude_m=100.0,
        down_mps=10.0,
        lookahead_s=5.0,
        min_relative_alt_m=15.0,
    ) == 50.0
    assert module.plane_target_altitude(
        relative_altitude_m=40.0,
        down_mps=10.0,
        lookahead_s=5.0,
        min_relative_alt_m=15.0,
    ) == 15.0


def test_plane_altitude_offset_uses_local_offset_ned() -> None:
    module = load_tracking_module()
    connection = FakeConnection()

    module.send_plane_altitude_offset(connection, 1, 1, 0.5)

    call = connection.mav.local_ned_calls[-1]
    assert call[3] == mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED
    assert call[7] == 0.5


def test_plane_attitude_uses_partial_roll_and_pitch_mask() -> None:
    module = load_tracking_module()
    connection = FakeConnection()

    module.send_plane_attitude(connection, 1, 1, 18.0, -12.0, 0.75)

    call = connection.mav.attitude_target_calls[-1]
    type_mask = call[3]
    assert not type_mask & mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
    assert not type_mask & mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
    assert type_mask & mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE
    assert not type_mask & mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE
    assert type_mask & mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE
    assert call[8] == 0.75
