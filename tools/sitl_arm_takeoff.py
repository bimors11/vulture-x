#!/usr/bin/env python3
"""Guarded ArduPilot SITL arm and takeoff helper."""

from __future__ import annotations

import argparse
import os
import sys
import time

from mavlink_endpoint import open_mavlink_connection
from pymavlink import mavutil


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mavlink",
        default=os.environ.get("VULTURE_X_MAVLINK", "udpin:0.0.0.0:14550"),
    )
    parser.add_argument(
        "--altitude-m",
        type=float,
        default=(
            float(os.environ["VULTURE_X_TAKEOFF_ALT_M"])
            if "VULTURE_X_TAKEOFF_ALT_M" in os.environ
            else None
        ),
    )
    parser.add_argument(
        "--vehicle",
        choices=("auto", "quad", "plane"),
        default="auto",
        help="Vehicle takeoff profile. auto uses the heartbeat MAV_TYPE.",
    )
    parser.add_argument(
        "--source-system",
        type=int,
        default=int(os.environ.get("VULTURE_X_MAVLINK_SOURCE_SYSTEM", "255")),
        help=(
            "MAVLink source system id for this helper. ArduPilot accepts RC "
            "override only from a system id allowed by MAV_GCS_SYSID; 255 is "
            "the ArduPilot default GCS id."
        ),
    )
    parser.add_argument(
        "--source-component",
        type=int,
        default=int(os.environ.get("VULTURE_X_MAVLINK_SOURCE_COMPONENT", "192")),
    )
    parser.add_argument("--timeout-s", type=float, default=45.0)
    return parser.parse_args()


def wait_mode(connection: mavutil.mavfile, mode_name: str, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        message = connection.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if message is None:
            continue
        current_mode = mavutil.mode_string_v10(message)
        if current_mode == mode_name:
            return True
    return False


def wait_armed(connection: mavutil.mavfile, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        connection.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if connection.motors_armed():
            return True
    return False


def set_parameter(
    connection: mavutil.mavfile,
    name: str,
    value: float,
    timeout_s: float,
) -> bool:
    encoded_name = name.encode("ascii")
    connection.mav.param_set_send(
        connection.target_system,
        connection.target_component,
        encoded_name,
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        message = connection.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if message is None:
            continue
        param_id = message.param_id
        if isinstance(param_id, bytes):
            param_id = param_id.decode("ascii", errors="ignore")
        if param_id.rstrip("\x00") == name and abs(float(message.param_value) - value) < 0.1:
            return True
    return False


def verify_sitl_operator_enable() -> None:
    if os.environ.get("VULTURE_X_ALLOW_SITL_ARM") != "1":
        raise RuntimeError(
            "Refusing to arm. Set VULTURE_X_ALLOW_SITL_ARM=1 for local SITL-only takeoff."
        )


def resolve_vehicle_profile(args: argparse.Namespace, heartbeat: object) -> str | None:
    if args.vehicle != "auto":
        return str(args.vehicle)
    if heartbeat.type == mavutil.mavlink.MAV_TYPE_QUADROTOR:
        return "quad"
    if heartbeat.type == mavutil.mavlink.MAV_TYPE_FIXED_WING:
        return "plane"
    return None


def arm_vehicle(connection: mavutil.mavfile, timeout_s: float) -> bool:
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    return wait_armed(connection, timeout_s)


def rc_override(connection: mavutil.mavfile, channels: dict[int, int]) -> None:
    values = [0] * 8
    for channel, pwm in channels.items():
        if channel < 1 or channel > 8:
            raise ValueError(f"RC override channel out of range: {channel}")
        values[channel - 1] = pwm
    connection.mav.rc_channels_override_send(
        connection.target_system,
        connection.target_component,
        *values,
    )


def latest_vfr_hud(connection: mavutil.mavfile, timeout_s: float) -> object | None:
    deadline = time.monotonic() + timeout_s
    latest = None
    while time.monotonic() < deadline:
        message = connection.recv_match(type="VFR_HUD", blocking=True, timeout=1)
        if message is not None:
            latest = message
            break
    return latest


def wait_groundspeed(connection: mavutil.mavfile, minimum_mps: float, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        message = connection.recv_match(type="VFR_HUD", blocking=True, timeout=1)
        if message is not None and float(message.groundspeed) >= minimum_mps:
            return True
    return False


def wait_relative_altitude(
    connection: mavutil.mavfile,
    minimum_altitude_m: float,
    timeout_s: float,
) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        message = connection.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
        if message is not None and float(message.relative_alt) / 1000.0 >= minimum_altitude_m:
            return True
    return False


def quad_takeoff(connection: mavutil.mavfile, altitude_m: float, timeout_s: float) -> int:
    mode_mapping = connection.mode_mapping()
    guided_mode = mode_mapping.get("GUIDED")
    if guided_mode is None:
        print("sitl_takeoff_status=failed reason=guided_mode_unavailable", flush=True)
        return 1

    print("sitl_takeoff_status=setting_mode vehicle=quad mode=GUIDED", flush=True)
    connection.set_mode(guided_mode)
    if not wait_mode(connection, "GUIDED", timeout_s):
        print("sitl_takeoff_status=failed reason=guided_mode_timeout", flush=True)
        return 1

    print("sitl_takeoff_status=arming vehicle=quad", flush=True)
    if not arm_vehicle(connection, timeout_s):
        print("sitl_takeoff_status=failed reason=arm_timeout", flush=True)
        return 1

    print(f"sitl_takeoff_status=takeoff vehicle=quad altitude_m={altitude_m}", flush=True)
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        altitude_m,
    )
    ack = connection.recv_match(type="COMMAND_ACK", blocking=True, timeout=timeout_s)
    if ack is None or ack.command != mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
        print("sitl_takeoff_status=warning reason=takeoff_ack_not_observed", flush=True)
    else:
        result = mavutil.mavlink.enums["MAV_RESULT"].get(ack.result)
        result_name = result.name if result else str(ack.result)
        print(f"sitl_takeoff_ack={result_name}", flush=True)
    print("sitl_takeoff_status=commanded vehicle=quad", flush=True)
    return 0


def plane_takeoff(connection: mavutil.mavfile, altitude_m: float, timeout_s: float) -> int:
    mode_mapping = connection.mode_mapping()
    fbwa_mode = mode_mapping.get("FBWA")
    if fbwa_mode is None:
        print("sitl_takeoff_status=failed reason=fbwa_mode_unavailable", flush=True)
        return 1
    circle_mode = mode_mapping.get("CIRCLE")

    print("sitl_takeoff_status=setting_mode vehicle=plane mode=FBWA", flush=True)
    connection.set_mode(fbwa_mode)
    if not wait_mode(connection, "FBWA", timeout_s):
        print("sitl_takeoff_status=failed reason=fbwa_mode_timeout", flush=True)
        return 1

    print("sitl_takeoff_status=arming vehicle=plane", flush=True)
    if not arm_vehicle(connection, timeout_s):
        print("sitl_takeoff_status=failed reason=arm_timeout", flush=True)
        return 1

    print("sitl_takeoff_status=rolling vehicle=plane stage=official_zephyr_rc3_1800", flush=True)
    rc_override(connection, {1: 1500, 2: 1500, 3: 1800, 4: 1500})
    if not wait_groundspeed(connection, 6.0, timeout_s):
        hud = latest_vfr_hud(connection, 2.0)
        groundspeed = getattr(hud, "groundspeed", "unknown") if hud is not None else "unknown"
        print(
            f"sitl_takeoff_status=failed reason=groundspeed_timeout "
            f"stage=official_zephyr_rc3_1800 minimum_mps=6.0 current_mps={groundspeed}",
            flush=True,
        )
        return 1

    if circle_mode is not None:
        print("sitl_takeoff_status=setting_mode vehicle=plane mode=CIRCLE", flush=True)
        connection.set_mode(circle_mode)
        if not wait_mode(connection, "CIRCLE", timeout_s):
            print("sitl_takeoff_status=warning reason=circle_mode_timeout", flush=True)

    print("sitl_takeoff_status=climbing vehicle=plane throttle_pwm=1800", flush=True)
    if not wait_relative_altitude(connection, altitude_m, timeout_s):
        print(
            f"sitl_takeoff_status=warning reason=altitude_timeout "
            f"vehicle=plane target_altitude_m={altitude_m}",
            flush=True,
        )
    rc_override(connection, {1: 1500, 2: 1500, 3: 1800, 4: 1500})
    print(
        f"sitl_takeoff_status=commanded vehicle=plane mode=CIRCLE altitude_m={altitude_m}",
        flush=True,
    )
    return 0


def main() -> int:
    args = parse_args()
    verify_sitl_operator_enable()

    connection = open_mavlink_connection(
        args.mavlink,
        source_system=args.source_system,
        source_component=args.source_component,
        autoreconnect=False,
    )
    heartbeat = connection.wait_heartbeat(timeout=args.timeout_s)
    if heartbeat is None:
        print("sitl_takeoff_status=failed reason=no_heartbeat", flush=True)
        return 1

    if heartbeat.autopilot != mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
        print(
            f"sitl_takeoff_status=failed reason=unexpected_autopilot value={heartbeat.autopilot}",
            flush=True,
        )
        return 1
    vehicle = resolve_vehicle_profile(args, heartbeat)
    if vehicle is None:
        print(
            f"sitl_takeoff_status=failed reason=unexpected_vehicle_type value={heartbeat.type}",
            flush=True,
        )
        return 1
    if args.vehicle != "auto" and args.vehicle != vehicle:
        print(
            f"sitl_takeoff_status=failed reason=vehicle_profile_mismatch "
            f"requested={args.vehicle} detected={vehicle}",
            flush=True,
        )
        return 1

    altitude_m = args.altitude_m
    if altitude_m is None:
        altitude_m = 5.0 if vehicle == "quad" else 50.0

    if vehicle == "quad":
        return quad_takeoff(connection, altitude_m, args.timeout_s)
    return plane_takeoff(connection, altitude_m, args.timeout_s)


if __name__ == "__main__":
    sys.exit(main())
