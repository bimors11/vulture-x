import builtins
import importlib.util
import os
import sys
from pathlib import Path
from types import ModuleType

import cv2
import numpy as np
import pytest

MAVLINK = __import__("pymavlink.mavutil", fromlist=["mavlink"]).mavlink


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
        self.rc_override_calls: list[tuple[object, ...]] = []
        self.param_request_read_calls: list[tuple[object, ...]] = []
        self.heartbeat_send_calls: list[tuple[object, ...]] = []

    def heartbeat_send(self, *args: object) -> None:
        self.heartbeat_send_calls.append(args)

    def command_int_send(self, *args: object) -> None:
        self.command_int_calls.append(args)

    def set_position_target_local_ned_send(self, *args: object) -> None:
        self.local_ned_calls.append(args)

    def set_attitude_target_send(self, *args: object) -> None:
        self.attitude_target_calls.append(args)

    def rc_channels_override_send(self, *args: object) -> None:
        self.rc_override_calls.append(args)

    def param_request_read_send(self, *args: object) -> None:
        self.param_request_read_calls.append(args)


class FakeParamMessage:
    def __init__(self, name: str, value: float) -> None:
        self.param_id = name.encode("ascii")
        self.param_value = value


class FakeHeartbeatMessage:
    def __init__(self, timestamp_s: float) -> None:
        self._timestamp = timestamp_s


class FakeCachedHeartbeatConnection:
    def __init__(self, heartbeat: FakeHeartbeatMessage) -> None:
        self.messages = {"HEARTBEAT": heartbeat}

    def recv_match(
        self,
        *,
        type: str | list[str],
        blocking: bool,
        timeout: float,
    ) -> object | None:
        del type, blocking, timeout
        return None


class FakeConnection:
    def __init__(self) -> None:
        self.mav = FakeMav()
        self.target_system = 1
        self.target_component = 1
        self.messages: list[object] = []

    def recv_match(
        self,
        *,
        type: str | list[str],
        blocking: bool,
        timeout: float,
    ) -> object | None:
        del type, blocking, timeout
        return self.messages.pop(0) if self.messages else None


class FakePlaneConnection(FakeConnection):
    def __init__(self, *, mode: str, armed: bool) -> None:
        super().__init__()
        self._mode = mode
        self._armed = armed
        self.mode_set: int | None = None
        self.heartbeat_mode = mode

    def wait_heartbeat(self, timeout: float) -> object:
        del timeout
        return type(
            "Heartbeat",
            (),
            {
                "mode_name": "MANUAL",
                "autopilot": MAVLINK.MAV_AUTOPILOT_ARDUPILOTMEGA,
                "type": MAVLINK.MAV_TYPE_FIXED_WING,
                "base_mode": (
                    MAVLINK.MAV_MODE_FLAG_SAFETY_ARMED if self._armed else 0
                ),
            },
        )()

    def mode_mapping(self) -> dict[str, int]:
        return {"FBWA": 5, "MANUAL": 0}

    def set_mode(self, mode_number: int) -> None:
        self.mode_set = mode_number
        self.heartbeat_mode = "FBWA"

    def recv_match(
        self,
        *,
        type: str | list[str],
        blocking: bool,
        timeout: float,
    ) -> object | None:
        del blocking, timeout
        if self.heartbeat_mode == "FBWA":
            return builtins.type(
                "Heartbeat",
                (),
                {
                    "mode_name": "FBWA",
                    "autopilot": MAVLINK.MAV_AUTOPILOT_ARDUPILOTMEGA,
                    "type": MAVLINK.MAV_TYPE_FIXED_WING,
                    "base_mode": MAVLINK.MAV_MODE_FLAG_SAFETY_ARMED,
                },
            )()
        return None


def test_poll_heartbeat_accepts_new_cached_mavlink_heartbeat() -> None:
    module = load_tracking_module()
    heartbeat = FakeHeartbeatMessage(123.0)
    connection = FakeCachedHeartbeatConnection(heartbeat)

    message, timestamp_s = module.poll_heartbeat(connection, 122.0)

    assert message is heartbeat
    assert timestamp_s == 123.0


def test_fixed_wing_guided_change_helpers_are_removed() -> None:
    module = load_tracking_module()

    assert not hasattr(module, "send_plane_speed")
    assert not hasattr(module, "send_plane_altitude")
    assert not hasattr(module, "send_plane_altitude_offset")
    assert not hasattr(module, "send_plane_heading")
    assert not hasattr(module, "send_plane_attitude")
    assert not hasattr(module, "plane_target_altitude")


def test_tracking_timeout_defaults_to_run_until_stopped(monkeypatch) -> None:
    module = load_tracking_module()
    monkeypatch.setattr(sys, "argv", ["sitl_track_target.py", "--enable-guidance"])

    args = module.parse_args()

    assert args.timeout_s == 0.0
    assert args.mavlink_heartbeat_timeout_s == 3.0


def test_heartbeat_timeout_flag_is_configurable(monkeypatch) -> None:
    module = load_tracking_module()
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "sitl_track_target.py",
            "--enable-guidance",
            "--mavlink-heartbeat-timeout-s",
            "7.5",
        ],
    )

    args = module.parse_args()

    assert args.mavlink_heartbeat_timeout_s == 7.5


def test_plane_rc_attitude_maps_negative_pitch_to_lower_elevator_pwm() -> None:
    module = load_tracking_module()
    connection = FakeConnection()

    module.send_plane_rc_attitude(
        connection,
        roll_deg=17.5,
        pitch_deg=-17.5,
        max_roll_deg=35.0,
        max_pitch_deg=35.0,
        throttle=0.75,
    )

    call = connection.mav.rc_override_calls[-1]
    assert call[0] == 1
    assert call[1] == 1
    assert call[2] == 1750
    assert call[3] == 1250
    assert call[4] == 1750
    assert call[5] == 1500


def test_plane_rc_attitude_can_reverse_pitch_channel() -> None:
    module = load_tracking_module()
    connection = FakeConnection()

    module.send_plane_rc_attitude(
        connection,
        roll_deg=0.0,
        pitch_deg=-17.5,
        max_roll_deg=35.0,
        max_pitch_deg=35.0,
        throttle=0.55,
        pitch_rc_reversed=True,
    )

    call = connection.mav.rc_override_calls[-1]
    assert call[3] == 1750


def test_plane_rc_attitude_does_not_apply_reversal_twice_with_calibration() -> None:
    module = load_tracking_module()
    connection = FakeConnection()

    module.send_plane_rc_attitude(
        connection,
        roll_deg=0.0,
        pitch_deg=-17.5,
        max_roll_deg=35.0,
        max_pitch_deg=35.0,
        throttle=0.55,
        pitch_rc_reversed=True,
        rc_calibration=module.RcCalibrationSet(
            pitch=module.RcCalibration(reversed=True),
        ),
    )

    call = connection.mav.rc_override_calls[-1]
    assert call[3] == 1750


def test_plane_throttle_reduces_for_nose_down_pitch() -> None:
    module = load_tracking_module()

    assert module.plane_throttle_for_pitch(
        pitch_deg=0.0,
        max_pitch_deg=35.0,
        neutral_throttle=0.60,
        min_throttle=0.35,
        max_throttle=0.80,
    ) == 0.60
    assert module.plane_throttle_for_pitch(
        pitch_deg=-35.0,
        max_pitch_deg=35.0,
        neutral_throttle=0.60,
        min_throttle=0.35,
        max_throttle=0.80,
    ) == 0.35
    assert module.plane_throttle_for_pitch(
        pitch_deg=35.0,
        max_pitch_deg=35.0,
        neutral_throttle=0.60,
        min_throttle=0.35,
        max_throttle=0.80,
    ) == 0.80


def test_plane_visual_steering_defaults_to_airspeed_governor(monkeypatch) -> None:
    module = load_tracking_module()
    monkeypatch.setattr(sys, "argv", ["sitl_track_target.py", "--enable-guidance"])

    args = module.parse_args()

    assert module.PLANE_TARGET_AIRSPEED_MPS == 20.0
    assert module.PLANE_CRUISE_THROTTLE == 0.55
    assert args.plane_airspeed_mps == 20.0
    assert args.plane_throttle == 0.55
    assert args.vertical_gain == 52.0
    assert args.plane_centering_gain == 1.55
    assert args.plane_near_centering_gain == 2.65
    assert args.plane_damping_gain == 0.14
    assert args.plane_near_damping_gain == 0.30
    assert args.plane_error_deadband == 0.015
    assert args.plane_lead_s == 0.0
    assert args.plane_roll_gain_scale == 1.75
    assert args.plane_pitch_gain_scale == 1.20
    assert args.plane_pitch_near_gain_scale == 1.60
    assert args.plane_far_control_scale == 0.72
    assert args.plane_pitch_below_center_boost == 0.25
    assert args.plane_near_control_scale == 1.0
    assert args.plane_near_pitch_down_limit_deg == 40.0
    assert args.plane_near_throttle_reduction == 0.0
    assert args.plane_max_pitch_step_deg == 2.0
    assert args.plane_max_roll_step_deg == 6.0
    assert args.plane_loss_hold_s == 1.5
    assert args.tracking_mode == "red"
    assert args.read_plane_params is True
    assert args.surface_test is False


def test_plane_tuning_file_applies_valid_updates_and_keeps_last_valid(
    tmp_path: Path,
    monkeypatch,
) -> None:
    module = load_tracking_module()
    monkeypatch.setattr(sys, "argv", ["sitl_track_target.py", "--enable-guidance"])
    args = module.parse_args()
    tuning = module.plane_tuning_from_args(args)
    tuning_path = tmp_path / "tracking_tuning.json"
    tuning_path.write_text(
        '{"revision":4,"values":{"plane_centering_gain":2.4,"plane_proximity_far_size":0.04}}',
        encoding="utf-8",
    )

    state, updated = module.update_tuning_from_file(
        module.TuningFileState(tuning_path),
        tuning,
    )

    assert state.mtime_ns is not None
    assert updated.revision == 4
    assert updated.plane_centering_gain == 2.4
    assert updated.plane_proximity_far_size == 0.04

    tuning_path.write_text("{not-json", encoding="utf-8")
    _state, still_valid = module.update_tuning_from_file(state, updated)

    assert still_valid == updated


def test_rc_override_sysid_high_zero_uses_single_gcs_id() -> None:
    module = load_tracking_module()

    assert (
        module.validate_rc_override_acceptance(
            {"MAV_OPTIONS": 1.0, "MAV_GCS_SYSID": 255.0, "MAV_GCS_SYSID_HI": 0.0},
            255,
        )
        is None
    )
    assert module.validate_rc_override_acceptance(
        {"MAV_OPTIONS": 1.0, "MAV_GCS_SYSID": 255.0},
        42,
    ) == (
        "rc_override_sysid_mismatch"
    )


def test_rc_override_sysid_is_not_enforced_when_mav_options_allows_any_gcs() -> None:
    module = load_tracking_module()

    assert (
        module.validate_rc_override_acceptance(
            {"MAV_OPTIONS": 0.0, "MAV_GCS_SYSID": 255.0},
            42,
        )
        is None
    )


def test_rc_override_validation_blocks_disabled_or_infinite_override() -> None:
    module = load_tracking_module()

    assert (
        module.validate_rc_override_acceptance({"RC_OPTIONS": 2.0}, 255)
        == "rc_override_disabled"
    )
    assert (
        module.validate_rc_override_acceptance({"RC_OVERRIDE_TIME": 0.0}, 255)
        == "rc_override_disabled"
    )
    assert (
        module.validate_rc_override_acceptance({"RC_OVERRIDE_TIME": -1.0}, 255)
        == "rc_override_disabled"
    )


def test_plane_visual_steering_accepts_red_tracking_mode(monkeypatch) -> None:
    module = load_tracking_module()
    monkeypatch.setattr(
        sys,
        "argv",
        ["sitl_track_target.py", "--enable-guidance", "--tracking-mode", "red"],
    )

    args = module.parse_args()

    assert args.tracking_mode == "red"


def test_plane_surface_test_argument_is_explicit(monkeypatch) -> None:
    module = load_tracking_module()
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "sitl_track_target.py",
            "--enable-guidance",
            "--surface-test",
            "--surface-test-throttle",
            "0.8",
        ],
    )

    args = module.parse_args()

    assert args.surface_test is True
    assert args.surface_test_throttle == 0.8


def test_head_tracking_mode_uses_operator_selection(monkeypatch) -> None:
    module = load_tracking_module()
    monkeypatch.setattr(
        sys,
        "argv",
        ["sitl_track_target.py", "--enable-guidance", "--tracking-mode", "head"],
    )

    args = module.parse_args()

    assert args.tracking_mode == "head"


def test_reassert_plane_mode_helper_is_removed() -> None:
    module = load_tracking_module()

    assert not hasattr(module, "should_reassert_plane_mode")


def test_verify_connection_accepts_plane_from_any_initial_mode(monkeypatch) -> None:
    module = load_tracking_module()
    connection = FakePlaneConnection(mode="MANUAL", armed=True)
    args = type(
        "Args",
        (),
        {
            "timeout_s": 2.0,
            "vehicle": "plane",
            "simulator_mode": False,
            "surface_test": False,
        },
    )()
    monkeypatch.setattr(
        module.mavutil,
        "mode_string_v10",
        lambda message: getattr(message, "mode_name", "FBWA"),
    )

    result = module.verify_connection(connection, args)

    assert result == (1, 1, "plane")
    assert connection.mode_set is None


def test_request_plane_fbwa_changes_mode_once_and_waits_for_confirmation(monkeypatch) -> None:
    module = load_tracking_module()
    connection = FakePlaneConnection(mode="MANUAL", armed=True)
    monkeypatch.setattr(
        module.mavutil,
        "mode_string_v10",
        lambda message: getattr(message, "mode_name", "FBWA"),
    )

    assert module.request_plane_fbwa(connection, "MANUAL", timeout_s=0.5) is True
    assert connection.mode_set == 5


def test_surface_test_allows_unarmed_plane_startup(monkeypatch) -> None:
    module = load_tracking_module()
    connection = FakePlaneConnection(mode="MANUAL", armed=False)
    args = type(
        "Args",
        (),
        {
            "timeout_s": 2.0,
            "vehicle": "plane",
            "simulator_mode": False,
            "surface_test": True,
        },
    )()
    monkeypatch.setattr(
        module.mavutil,
        "mode_string_v10",
        lambda message: getattr(message, "mode_name", "FBWA"),
    )

    result = module.verify_connection(connection, args)

    assert result == (1, 1, "plane")
    assert connection.mode_set is None


def test_apply_deadband_zeroes_small_image_error() -> None:
    module = load_tracking_module()

    assert module.apply_deadband(0.02, 0.035) == 0.0
    assert module.apply_deadband(-0.02, 0.035) == 0.0
    assert module.apply_deadband(0.05, 0.035) == 0.05


def test_near_target_control_scale_tapers_to_minimum() -> None:
    module = load_tracking_module()

    assert module.near_target_control_scale(0.0, 0.45) == 1.0
    assert module.near_target_control_scale(1.0, 0.45) == pytest.approx(0.45)


def test_far_target_control_scale_starts_soft_and_releases_near() -> None:
    module = load_tracking_module()

    assert module.far_target_control_scale(0.0, 0.55) == pytest.approx(0.55)
    assert module.far_target_control_scale(0.5, 0.55) == pytest.approx(0.775)
    assert module.far_target_control_scale(1.0, 0.55) == pytest.approx(1.0)


def test_scheduled_gain_increases_near_target() -> None:
    module = load_tracking_module()

    assert module.scheduled_gain(1.15, 2.15, 0.0) == pytest.approx(1.15)
    assert module.scheduled_gain(1.15, 2.15, 0.5) == pytest.approx(1.65)
    assert module.scheduled_gain(1.15, 2.15, 1.0) == pytest.approx(2.15)


def test_damped_axis_error_smooths_toward_new_error() -> None:
    module = load_tracking_module()

    assert module.damped_axis_error(0.30, 0.20, 1.0, 0.10) == pytest.approx(
        0.2909090909
    )
    assert module.damped_axis_error(0.20, 0.30, 1.0, 0.10) == pytest.approx(
        0.2090909091
    )


def test_read_plane_parameters_requests_named_params() -> None:
    module = load_tracking_module()
    connection = FakeConnection()
    connection.messages = [
        FakeParamMessage("LIM_ROLL_CD", 3500.0),
        FakeParamMessage("PTCH2SRV_P", 0.7),
    ]

    values = module.read_plane_parameters(
        connection,
        1,
        1,
        ("LIM_ROLL_CD", "PTCH2SRV_P"),
        timeout_per_param_s=0.02,
    )

    assert values == {"LIM_ROLL_CD": 3500.0, "PTCH2SRV_P": 0.7}
    assert len(connection.mav.param_request_read_calls) == 2


def test_plane_parameter_cache_is_endpoint_scoped(tmp_path: Path) -> None:
    module = load_tracking_module()
    cache_path = tmp_path / "plane_params.json"

    module.save_plane_param_cache(
        cache_path,
        "udpcl:192.168.144.12:19856",
        {"LIM_ROLL_CD": 3500.0, "RCMAP_ROLL": 1.0},
    )

    assert module.load_plane_param_cache(cache_path, "udpcl:192.168.144.12:19856") == {
        "LIM_ROLL_CD": 3500.0,
        "RCMAP_ROLL": 1.0,
    }
    assert module.load_plane_param_cache(cache_path, "udpcl:192.168.144.99:19856") is None


def test_plane_response_model_uses_aircraft_limits_and_time_constant(monkeypatch) -> None:
    module = load_tracking_module()
    monkeypatch.setattr(sys, "argv", ["sitl_track_target.py", "--enable-guidance"])
    args = module.parse_args()

    model = module.plane_response_model_from_params(
        args,
        {
            "LIM_ROLL_CD": 3200.0,
            "LIM_PITCH_MAX": 2500.0,
            "LIM_PITCH_MIN": -1800.0,
            "TRIM_THROTTLE": 48.0,
            "THR_MIN": 20.0,
            "THR_MAX": 75.0,
            "ARSPD_FBW_MIN": 17.0,
            "ARSPD_FBW_MAX": 19.0,
            "PTCH2SRV_TCONST": 0.4,
            "RCMAP_ROLL": 4.0,
            "RCMAP_PITCH": 2.0,
            "RCMAP_THROTTLE": 1.0,
            "RCMAP_YAW": 3.0,
            "RC2_REVERSED": 1.0,
        },
    )

    assert model.max_roll_deg == 32.0
    assert model.max_pitch_up_deg == 25.0
    assert model.max_pitch_down_deg == 18.0
    assert model.cruise_throttle == 0.48
    assert model.min_throttle == 0.20
    assert model.max_throttle == 0.75
    assert model.target_airspeed_mps == 19.0
    assert 0.08 <= model.pitch_filter_alpha <= args.plane_pitch_filter_alpha
    assert model.pitch_rc_reversed is True
    assert model.roll_channel == 4
    assert model.pitch_channel == 2
    assert model.throttle_channel == 1
    assert model.yaw_channel == 3


def test_plane_response_model_uses_mapped_channel_calibration_and_reversal(monkeypatch) -> None:
    module = load_tracking_module()
    monkeypatch.setattr(sys, "argv", ["sitl_track_target.py", "--enable-guidance"])
    args = module.parse_args()

    model = module.plane_response_model_from_params(
        args,
        {
            "RCMAP_ROLL": 5.0,
            "RC5_MIN": 982.0,
            "RC5_TRIM": 1493.0,
            "RC5_MAX": 2018.0,
            "RC5_REVERSED": 1.0,
        },
    )

    assert model.roll_channel == 5
    assert model.rc_calibration.roll.minimum == 982
    assert model.rc_calibration.roll.trim == 1493
    assert model.rc_calibration.roll.maximum == 2018
    assert model.rc_calibration.roll.reversed is True


def test_plane_response_model_keeps_explicit_fixed_throttle(monkeypatch) -> None:
    module = load_tracking_module()
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "sitl_track_target.py",
            "--enable-guidance",
            "--plane-throttle",
            "0.8",
            "--plane-min-throttle",
            "0.8",
            "--plane-max-throttle",
            "0.8",
        ],
    )
    args = module.parse_args()

    model = module.plane_response_model_from_params(
        args,
        {
            "TRIM_THROTTLE": 45.0,
            "THR_MIN": 10.0,
            "THR_MAX": 90.0,
        },
    )

    assert model.cruise_throttle == pytest.approx(0.8)
    assert model.min_throttle == pytest.approx(0.8)
    assert model.max_throttle == pytest.approx(0.8)


def test_plane_response_model_accepts_live_tuning_without_rate_hz(monkeypatch) -> None:
    module = load_tracking_module()
    monkeypatch.setattr(sys, "argv", ["sitl_track_target.py", "--enable-guidance"])
    args = module.parse_args()
    tuning = module.plane_tuning_from_args(args)

    model = module.plane_response_model_from_params(
        tuning,
        {
            "PTCH2SRV_TCONST": 0.4,
        },
    )

    assert 0.08 <= model.pitch_filter_alpha <= tuning.plane_pitch_filter_alpha


def test_plane_rc_attitude_uses_rcmap_channels() -> None:
    module = load_tracking_module()
    connection = FakeConnection()

    module.send_plane_rc_attitude(
        connection,
        roll_deg=17.5,
        pitch_deg=-17.5,
        max_roll_deg=35.0,
        max_pitch_deg=35.0,
        throttle=0.75,
        roll_channel=4,
        pitch_channel=2,
        throttle_channel=1,
        yaw_channel=3,
    )

    call = connection.mav.rc_override_calls[-1]
    assert call[2] == 1750
    assert call[3] == 1250
    assert call[4] == 1500
    assert call[5] == 1750


def test_surface_test_min_deflection_makes_small_pitch_visible() -> None:
    module = load_tracking_module()

    assert module.enforce_visible_surface_deflection(
        command_deg=1.5,
        error=-0.03,
        min_deflection_deg=8.0,
        limit_deg=40.0,
        invert_error_sign=True,
    ) == 8.0
    assert module.enforce_visible_surface_deflection(
        command_deg=-1.5,
        error=0.03,
        min_deflection_deg=8.0,
        limit_deg=40.0,
        invert_error_sign=True,
    ) == -8.0
    assert module.enforce_visible_surface_deflection(
        command_deg=-2.0,
        error=0.20,
        min_deflection_deg=8.0,
        limit_deg=40.0,
    ) == 8.0
    assert module.enforce_visible_surface_deflection(
        command_deg=12.0,
        error=-0.20,
        min_deflection_deg=8.0,
        limit_deg=40.0,
    ) == -8.0


def test_plane_throttle_governor_reduces_when_fast_or_nose_down() -> None:
    module = load_tracking_module()

    cruise = module.plane_throttle_for_airspeed(
        airspeed_mps=20.0,
        target_airspeed_mps=20.0,
        cruise_throttle=0.55,
        min_throttle=0.25,
        max_throttle=0.80,
        airspeed_gain=0.04,
        pitch_deg=0.0,
        max_pitch_deg=40.0,
    )
    fast = module.plane_throttle_for_airspeed(
        airspeed_mps=25.0,
        target_airspeed_mps=20.0,
        cruise_throttle=0.55,
        min_throttle=0.25,
        max_throttle=0.80,
        airspeed_gain=0.04,
        pitch_deg=0.0,
        max_pitch_deg=40.0,
    )
    descending = module.plane_throttle_for_airspeed(
        airspeed_mps=25.0,
        target_airspeed_mps=20.0,
        cruise_throttle=0.55,
        min_throttle=0.25,
        max_throttle=0.80,
        airspeed_gain=0.04,
        pitch_deg=-40.0,
        max_pitch_deg=40.0,
    )

    assert cruise == pytest.approx(0.55)
    assert fast == pytest.approx(0.35)
    assert descending == pytest.approx(0.25)


def test_plane_throttle_governor_reduces_near_target() -> None:
    module = load_tracking_module()

    far = module.plane_throttle_for_airspeed(
        airspeed_mps=20.0,
        target_airspeed_mps=20.0,
        cruise_throttle=0.55,
        min_throttle=0.25,
        max_throttle=0.80,
        airspeed_gain=0.04,
        pitch_deg=0.0,
        max_pitch_deg=40.0,
        target_proximity=0.0,
        near_target_throttle_reduction=0.16,
    )
    near = module.plane_throttle_for_airspeed(
        airspeed_mps=20.0,
        target_airspeed_mps=20.0,
        cruise_throttle=0.55,
        min_throttle=0.25,
        max_throttle=0.80,
        airspeed_gain=0.04,
        pitch_deg=0.0,
        max_pitch_deg=40.0,
        target_proximity=1.0,
        near_target_throttle_reduction=0.16,
    )

    assert far == pytest.approx(0.55)
    assert near == pytest.approx(0.39)


def test_plane_pitch_command_softens_targets_below_center_when_near() -> None:
    module = load_tracking_module()

    far = module.plane_pitch_command(
        guided_error_y=0.25,
        vertical_gain=40.0,
        far_pitch_gain_scale=1.1,
        near_pitch_gain_scale=1.05,
        target_proximity=0.0,
        below_center_boost=0.25,
        max_pitch_deg=35.0,
    )
    near = module.plane_pitch_command(
        guided_error_y=0.25,
        vertical_gain=40.0 * 0.70,
        far_pitch_gain_scale=1.1,
        near_pitch_gain_scale=1.05,
        target_proximity=1.0,
        below_center_boost=0.25,
        max_pitch_deg=35.0,
    )

    assert near < 0.0
    assert abs(near) < abs(far)


def test_plane_pitch_command_keeps_above_center_unboosted() -> None:
    module = load_tracking_module()

    assert module.plane_pitch_command(
        guided_error_y=-0.2,
        vertical_gain=40.0,
        far_pitch_gain_scale=0.9,
        near_pitch_gain_scale=2.2,
        target_proximity=0.0,
        below_center_boost=0.8,
        max_pitch_deg=35.0,
    ) == pytest.approx(7.2)


def test_plane_pitch_command_limits_near_target_nose_down() -> None:
    module = load_tracking_module()

    assert module.plane_pitch_command(
        guided_error_y=0.4,
        vertical_gain=40.0,
        far_pitch_gain_scale=0.8,
        near_pitch_gain_scale=3.0,
        target_proximity=1.0,
        below_center_boost=0.7,
        max_pitch_deg=40.0,
        near_pitch_down_limit_deg=18.0,
    ) == pytest.approx(-18.0)


def test_damped_axis_error_stays_bounded_for_tracking_jitter() -> None:
    module = load_tracking_module()

    decreasing = module.damped_axis_error(
        error=-0.4,
        previous_error=0.4,
        dt_s=0.001,
        damping_gain=0.45,
    )
    increasing = module.damped_axis_error(
        error=0.4,
        previous_error=-0.4,
        dt_s=0.001,
        damping_gain=0.45,
    )

    assert -1.0 <= decreasing <= 1.0
    assert -1.0 <= increasing <= 1.0
    assert decreasing == pytest.approx(0.398, abs=0.001)
    assert increasing == pytest.approx(-0.398, abs=0.001)


def test_target_proximity_increases_with_bbox_size() -> None:
    module = load_tracking_module()

    far = module.target_proximity_from_bbox((10, 10, 10, 10), 640, 480)
    near = module.target_proximity_from_bbox((10, 10, 120, 90), 640, 480)

    assert far == 0.0
    assert near > 0.8


def test_damp_pitch_command_rate_limits_changes() -> None:
    module = load_tracking_module()

    assert module.damp_pitch_command(None, -30.0, 0.35, 5.0) == -30.0
    assert module.damp_pitch_command(0.0, -30.0, 0.35, 5.0) == -5.0


def test_stabilize_bbox_rejects_large_jump() -> None:
    module = load_tracking_module()

    assert module.stabilize_bbox(
        (100, 100, 30, 30),
        (500, 100, 30, 30),
        640,
        480,
    ) == (100, 100, 30, 30)


def test_bbox_transition_allows_stable_bbox_and_rejects_jump() -> None:
    module = load_tracking_module()

    assert module.bbox_transition_plausible(
        (100, 100, 30, 30),
        (101, 101, 30, 30),
        640,
        480,
        max_center_jump_norm=0.16,
        max_size_ratio=6.0,
    )
    assert not module.bbox_transition_plausible(
        (100, 100, 30, 30),
        (500, 100, 30, 30),
        640,
        480,
        max_center_jump_norm=0.16,
        max_size_ratio=6.0,
    )


def test_stabilize_bbox_smooths_small_motion() -> None:
    module = load_tracking_module()

    assert module.stabilize_bbox(
        (100, 100, 30, 30),
        (110, 104, 32, 32),
        640,
        480,
    ) == (103, 101, 31, 31)


def test_detector_backed_tracker_initializes_from_banner_detection() -> None:
    module = load_tracking_module()
    frame = np.zeros((120, 160, 3), dtype=np.uint8)
    cv2.rectangle(frame, (52, 32), (108, 88), (210, 0, 255), -1)
    cv2.line(frame, (60, 60), (100, 60), (10, 10, 10), 4)
    cv2.line(frame, (80, 40), (80, 80), (10, 10, 10), 4)
    cv2.line(frame, (62, 42), (98, 78), (10, 10, 10), 3)
    cv2.line(frame, (98, 42), (62, 78), (10, 10, 10), 3)

    tracker = module.DetectorBackedTracker(
        "TEMPLATE",
        min_area=80.0,
        detector=module.detect_banner_target,
    )
    bbox = tracker.bbox(frame)

    assert bbox is not None
    x, y, width, height = bbox
    assert x < 52
    assert y < 32
    assert x + width > 108
    assert y + height > 88


def test_detector_backed_tracker_initializes_from_red_detection() -> None:
    module = load_tracking_module()
    frame = np.zeros((120, 160, 3), dtype=np.uint8)
    cv2.rectangle(frame, (58, 36), (102, 92), (0, 0, 230), -1)

    tracker = module.DetectorBackedTracker("TEMPLATE", min_area=80.0)
    bbox = tracker.bbox(frame)

    assert bbox is not None
    x, y, width, height = bbox
    assert x < 58
    assert y < 36
    assert x + width > 102
    assert y + height > 92


class FakeSequenceTracker:
    def __init__(self) -> None:
        self.updates = [
            (False, (0, 0, 0, 0)),
            (False, (0, 0, 0, 0)),
            (True, (500, 100, 30, 30)),
        ]

    def init(self, frame: object, bbox: tuple[int, int, int, int]) -> bool:
        del frame, bbox
        return True

    def update(self, frame: object) -> tuple[bool, tuple[int, int, int, int]]:
        del frame
        return self.updates.pop(0)


def test_custom_selection_tracker_holds_bbox_through_short_misses(
    tmp_path: Path,
    monkeypatch,
) -> None:
    module = load_tracking_module()
    selection_file = tmp_path / "selection.json"
    selection_file.write_text(
        '{"mode":"custom","x":0.25,"y":0.25,"width":0.1,"height":0.1}',
        encoding="utf-8",
    )
    monkeypatch.setattr(module, "create_tracker", lambda _name: FakeSequenceTracker())
    frame = np.zeros((100, 200, 3), dtype=np.uint8)

    tracker = module.CustomSelectionTracker("TEMPLATE", selection_file)

    initial = tracker.bbox(frame)
    assert initial == (50, 25, 20, 10)
    assert tracker.bbox(frame) == initial
    assert tracker.bbox(frame) == initial
    assert tracker.bbox(frame) == initial


def test_custom_selection_bbox_refines_to_salient_region(tmp_path: Path) -> None:
    module = load_tracking_module()
    selection_file = tmp_path / "selection.json"
    selection_file.write_text(
        '{"mode":"custom","x":0.1,"y":0.1,"width":0.8,"height":0.7}',
        encoding="utf-8",
    )
    frame = np.zeros((120, 200, 3), dtype=np.uint8)
    frame[:, :] = (40, 40, 40)
    cv2.rectangle(frame, (64, 38), (104, 86), (0, 0, 230), -1)

    bbox = module.selection_bbox(selection_file, frame)

    x, y, width, height = bbox
    assert x > 45
    assert y > 25
    assert x + width < 125
    assert y + height < 100


def test_head_selection_bbox_uses_same_roi_path(tmp_path: Path) -> None:
    module = load_tracking_module()
    selection_file = tmp_path / "selection.json"
    selection_file.write_text(
        '{"mode":"head","x":0.1,"y":0.1,"width":0.8,"height":0.7}',
        encoding="utf-8",
    )
    frame = np.zeros((120, 200, 3), dtype=np.uint8)
    frame[:, :] = (40, 40, 40)
    cv2.rectangle(frame, (64, 38), (104, 86), (0, 0, 230), -1)

    bbox = module.selection_bbox(selection_file, frame)

    x, y, width, height = bbox
    assert x > 45
    assert y > 25
    assert x + width < 125
    assert y + height < 100


def test_frame_directory_reader_skips_unstable_and_duplicate_frames(
    tmp_path: Path,
    monkeypatch,
) -> None:
    module = load_tracking_module()
    old_frame = tmp_path / "frame-000001.jpg"
    new_frame = tmp_path / "frame-000002.jpg"
    cv2.imwrite(str(old_frame), np.zeros((12, 16, 3), dtype=np.uint8))
    cv2.imwrite(str(new_frame), np.full((12, 16, 3), 255, dtype=np.uint8))
    now = 1000.0
    os.utime(old_frame, (now - 1.0, now - 1.0))
    os.utime(new_frame, (now, now))
    monkeypatch.setattr(module.time, "time", lambda: now)

    reader = module.FrameDirectoryReader(tmp_path)

    ok, frame, age_s = reader.read()
    assert ok is True
    assert frame is not None
    assert age_s is not None and age_s >= 1.0
    assert int(frame[0, 0, 0]) == 0

    ok, frame, _age_s = reader.read()
    assert ok is False
    assert frame is None


def test_perspective_corrected_error_keeps_center_and_edges_stable() -> None:
    module = load_tracking_module()

    assert module.perspective_correct_error(0.0, 70.0) == 0.0
    assert module.perspective_correct_error(0.5, 70.0) == 0.5
    assert module.perspective_correct_error(-0.5, 70.0) == -0.5
    assert module.perspective_correct_error(0.25, 70.0) > 0.25
