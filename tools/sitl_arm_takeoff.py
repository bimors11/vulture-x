#!/usr/bin/env python3
"""Guarded ArduPilot SITL GUIDED arm and takeoff helper."""

from __future__ import annotations

import argparse
import os
import sys
import time

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
        default=float(os.environ.get("VULTURE_X_TAKEOFF_ALT_M", "5")),
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


def verify_sitl_operator_enable() -> None:
    if os.environ.get("VULTURE_X_ALLOW_SITL_ARM") != "1":
        raise RuntimeError(
            "Refusing to arm. Set VULTURE_X_ALLOW_SITL_ARM=1 for local SITL-only takeoff."
        )


def main() -> int:
    args = parse_args()
    verify_sitl_operator_enable()

    connection = mavutil.mavlink_connection(
        args.mavlink,
        source_system=191,
        source_component=192,
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
    if heartbeat.type != mavutil.mavlink.MAV_TYPE_QUADROTOR:
        print(
            f"sitl_takeoff_status=failed reason=unexpected_vehicle_type value={heartbeat.type}",
            flush=True,
        )
        return 1

    mode_mapping = connection.mode_mapping()
    guided_mode = mode_mapping.get("GUIDED")
    if guided_mode is None:
        print("sitl_takeoff_status=failed reason=guided_mode_unavailable", flush=True)
        return 1

    print("sitl_takeoff_status=setting_mode mode=GUIDED", flush=True)
    connection.set_mode(guided_mode)
    if not wait_mode(connection, "GUIDED", args.timeout_s):
        print("sitl_takeoff_status=failed reason=guided_mode_timeout", flush=True)
        return 1

    print("sitl_takeoff_status=arming", flush=True)
    connection.arducopter_arm()
    if not wait_armed(connection, args.timeout_s):
        print("sitl_takeoff_status=failed reason=arm_timeout", flush=True)
        return 1

    print(f"sitl_takeoff_status=takeoff altitude_m={args.altitude_m}", flush=True)
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
        args.altitude_m,
    )
    ack = connection.recv_match(type="COMMAND_ACK", blocking=True, timeout=args.timeout_s)
    if ack is None or ack.command != mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
        print("sitl_takeoff_status=warning reason=takeoff_ack_not_observed", flush=True)
    else:
        result = mavutil.mavlink.enums["MAV_RESULT"].get(ack.result)
        result_name = result.name if result else str(ack.result)
        print(f"sitl_takeoff_ack={result_name}", flush=True)

    print("sitl_takeoff_status=commanded", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
