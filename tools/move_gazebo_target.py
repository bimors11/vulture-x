#!/usr/bin/env python3
"""Move the Gazebo visual target within a bounded local test area."""

from __future__ import annotations

import argparse
import math
import random
import signal
import subprocess
import sys
import time


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", default="vulture_x_test")
    parser.add_argument("--model", default="target_marker")
    parser.add_argument("--center-x", type=float, default=12.0)
    parser.add_argument("--center-y", type=float, default=-5.0)
    parser.add_argument("--center-z", type=float, default=6.0)
    parser.add_argument("--roll-deg", type=float, default=0.0)
    parser.add_argument("--pitch-deg", type=float, default=0.0)
    parser.add_argument("--yaw-offset-deg", type=float, default=0.0)
    parser.add_argument("--fixed-yaw-deg", type=float, default=None)
    parser.add_argument(
        "--pattern",
        choices=("loiter", "back_and_forth", "random"),
        default="loiter",
    )
    parser.add_argument("--radius-y-m", type=float, default=4.0)
    parser.add_argument("--radius-z-m", type=float, default=1.4)
    parser.add_argument("--period-s", type=float, default=5.0)
    parser.add_argument("--range-y-m", type=float, default=3.5)
    parser.add_argument("--range-z-m", type=float, default=1.2)
    parser.add_argument("--step-y-m", type=float, default=0.75)
    parser.add_argument("--step-z-m", type=float, default=0.35)
    parser.add_argument("--rate-hz", type=float, default=8.0)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--once", action="store_true")
    return parser.parse_args()


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def loiter_pose(
    elapsed_s: float,
    *,
    center_x: float,
    center_y: float,
    center_z: float,
    radius_y_m: float,
    radius_z_m: float,
    period_s: float,
) -> tuple[float, float, float, float]:
    if period_s <= 0:
        raise ValueError("period_s must be positive")
    phase = (2.0 * math.pi * elapsed_s) / period_s
    y = center_y + radius_y_m * math.cos(phase)
    z = center_z + radius_z_m * math.sin(phase)
    yaw = math.pi if math.sin(phase) >= 0 else 0.0
    return center_x, y, z, yaw


def back_and_forth_pose(
    elapsed_s: float,
    *,
    center_x: float,
    center_y: float,
    center_z: float,
    range_y_m: float,
    range_z_m: float,
    period_s: float,
) -> tuple[float, float, float, float]:
    if period_s <= 0:
        raise ValueError("period_s must be positive")
    phase = (2.0 * math.pi * elapsed_s) / period_s
    lateral = math.sin(phase)
    y = center_y + range_y_m * lateral
    z = center_z + range_z_m * math.sin(phase * 0.5)
    yaw = math.pi if math.cos(phase) >= 0 else 0.0
    return center_x, y, z, yaw


def euler_to_quaternion(
    roll_rad: float,
    pitch_rad: float,
    yaw_rad: float,
) -> tuple[float, float, float, float]:
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def set_pose(
    world: str,
    model: str,
    x: float,
    y: float,
    z: float,
    roll_rad: float,
    pitch_rad: float,
    yaw_rad: float,
) -> tuple[bool, str]:
    qx, qy, qz, qw = euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)
    request = (
        f'name: "{model}" '
        f"position {{ x: {x:.3f} y: {y:.3f} z: {z:.3f} }} "
        "orientation "
        f"{{ x: {qx:.6f} y: {qy:.6f} z: {qz:.6f} w: {qw:.6f} }}"
    )
    result = subprocess.run(
        [
            "gz",
            "service",
            "-s",
            f"/world/{world}/set_pose",
            "--reqtype",
            "gz.msgs.Pose",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "1000",
            "--req",
            request,
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    output = (result.stdout + result.stderr).strip()
    return result.returncode == 0, output


def main() -> int:
    args = parse_args()
    if args.rate_hz <= 0:
        print("target_motion_status=failed reason=rate_hz_must_be_positive", flush=True)
        return 2
    if args.period_s <= 0:
        print("target_motion_status=failed reason=period_s_must_be_positive", flush=True)
        return 2

    rng = random.Random(args.seed)
    running = True

    def stop(_signum: int, _frame: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    y = args.center_y
    z = args.center_z
    yaw = math.pi
    roll_rad = math.radians(args.roll_deg)
    pitch_rad = math.radians(args.pitch_deg)
    yaw_offset_rad = math.radians(args.yaw_offset_deg)
    fixed_yaw_rad = math.radians(args.fixed_yaw_deg) if args.fixed_yaw_deg is not None else None
    period_s = 1.0 / args.rate_hz
    failures = 0
    last_report = 0.0
    start = time.monotonic()

    if args.once:
        command_yaw = fixed_yaw_rad if fixed_yaw_rad is not None else math.pi + yaw_offset_rad
        ok, detail = set_pose(
            args.world,
            args.model,
            args.center_x,
            args.center_y,
            args.center_z,
            roll_rad,
            pitch_rad,
            command_yaw,
        )
        if not ok:
            print(
                "target_motion_status=failed "
                f"reason=gazebo_set_pose_unavailable detail={detail!r}",
                flush=True,
            )
            return 2
        print(
            "target_motion_status=set "
            f"pose={args.center_x:.2f},{args.center_y:.2f},{args.center_z:.2f} "
            f"yaw_rad={command_yaw:.2f}",
            flush=True,
        )
        return 0

    while running:
        elapsed_s = time.monotonic() - start
        if args.pattern == "loiter":
            x, y, z, yaw = loiter_pose(
                elapsed_s,
                center_x=args.center_x,
                center_y=args.center_y,
                center_z=args.center_z,
                radius_y_m=args.radius_y_m,
                radius_z_m=args.radius_z_m,
                period_s=args.period_s,
            )
        elif args.pattern == "back_and_forth":
            x, y, z, yaw = back_and_forth_pose(
                elapsed_s,
                center_x=args.center_x,
                center_y=args.center_y,
                center_z=args.center_z,
                range_y_m=args.range_y_m,
                range_z_m=args.range_z_m,
                period_s=args.period_s,
            )
        else:
            x = args.center_x
            y = clamp(
                y + rng.uniform(-args.step_y_m, args.step_y_m),
                args.center_y - args.range_y_m,
                args.center_y + args.range_y_m,
            )
            z = clamp(
                z + rng.uniform(-args.step_z_m, args.step_z_m),
                args.center_z - args.range_z_m,
                args.center_z + args.range_z_m,
            )
            yaw = math.pi

        command_yaw = fixed_yaw_rad if fixed_yaw_rad is not None else yaw + yaw_offset_rad
        ok, detail = set_pose(
            args.world,
            args.model,
            x,
            y,
            z,
            roll_rad,
            pitch_rad,
            command_yaw,
        )
        now = time.monotonic()
        if ok:
            failures = 0
            if now - last_report >= 2.0:
                print(
                    "target_motion_status=moving "
                    f"pattern={args.pattern} pose={x:.2f},{y:.2f},{z:.2f} yaw_rad={yaw:.2f}"
                )
                last_report = now
        else:
            failures += 1
            if failures % 5 == 1:
                print(
                    "target_motion_status=waiting "
                    f"reason=gazebo_set_pose_unavailable detail={detail!r}",
                    flush=True,
                )
            time.sleep(min(2.0, period_s))
            continue
        time.sleep(period_s)

    set_pose(
        args.world,
        args.model,
        args.center_x,
        args.center_y,
        args.center_z,
        roll_rad,
        pitch_rad,
        fixed_yaw_rad if fixed_yaw_rad is not None else math.pi + yaw_offset_rad,
    )
    print("target_motion_status=stopped", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
