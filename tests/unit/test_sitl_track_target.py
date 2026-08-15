import importlib.util
import sys
from pathlib import Path
from types import ModuleType

import cv2
import numpy as np
import pytest
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
        self.rc_override_calls: list[tuple[object, ...]] = []
        self.param_request_read_calls: list[tuple[object, ...]] = []

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
    assert args.plane_centering_gain == 1.15
    assert args.plane_near_centering_gain == 2.15
    assert args.plane_damping_gain == 0.22
    assert args.plane_near_damping_gain == 0.45
    assert args.plane_error_deadband == 0.015
    assert args.plane_roll_gain_scale == 1.35
    assert args.plane_pitch_gain_scale == 1.10
    assert args.plane_pitch_near_gain_scale == 1.45
    assert args.plane_far_control_scale == 0.55
    assert args.plane_pitch_below_center_boost == 0.25
    assert args.plane_near_control_scale == 1.0
    assert args.plane_near_pitch_down_limit_deg == 40.0
    assert args.plane_near_throttle_reduction == 0.0
    assert args.plane_max_pitch_step_deg == 2.0
    assert args.plane_max_roll_step_deg == 3.0
    assert args.plane_loss_hold_s == 1.5
    assert args.read_plane_params is True


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


def test_damped_axis_error_opposes_error_rate() -> None:
    module = load_tracking_module()

    assert module.damped_axis_error(0.30, 0.20, 1.0, 0.10) == pytest.approx(0.29)
    assert module.damped_axis_error(0.20, 0.30, 1.0, 0.10) == pytest.approx(0.21)


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
            "RCMAP_PITCH": 2.0,
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

    tracker = module.DetectorBackedTracker("TEMPLATE", min_area=80.0)
    bbox = tracker.bbox(frame)

    assert bbox is not None
    x, y, width, height = bbox
    assert x < 52
    assert y < 32
    assert x + width > 108
    assert y + height > 88


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


def test_perspective_corrected_error_keeps_center_and_edges_stable() -> None:
    module = load_tracking_module()

    assert module.perspective_correct_error(0.0, 70.0) == 0.0
    assert module.perspective_correct_error(0.5, 70.0) == 0.5
    assert module.perspective_correct_error(-0.5, 70.0) == -0.5
    assert module.perspective_correct_error(0.25, 70.0) > 0.25
