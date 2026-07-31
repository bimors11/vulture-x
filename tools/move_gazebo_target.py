#!/usr/bin/env python3
"""Move the Gazebo visual target within a bounded local test area."""

from __future__ import annotations

import argparse
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
    parser.add_argument("--range-y-m", type=float, default=2.0)
    parser.add_argument("--range-z-m", type=float, default=1.1)
    parser.add_argument("--step-y-m", type=float, default=0.45)
    parser.add_argument("--step-z-m", type=float, default=0.25)
    parser.add_argument("--rate-hz", type=float, default=2.0)
    parser.add_argument("--seed", type=int, default=None)
    return parser.parse_args()


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def set_pose(
    world: str,
    model: str,
    x: float,
    y: float,
    z: float,
) -> tuple[bool, str]:
    request = (
        f'name: "{model}" '
        f"position {{ x: {x:.3f} y: {y:.3f} z: {z:.3f} }} "
        "orientation { w: 1.0 }"
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

    rng = random.Random(args.seed)
    running = True

    def stop(_signum: int, _frame: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    y = args.center_y
    z = args.center_z
    period_s = 1.0 / args.rate_hz
    failures = 0
    last_report = 0.0

    while running:
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
        ok, detail = set_pose(args.world, args.model, args.center_x, y, z)
        now = time.monotonic()
        if ok:
            failures = 0
            if now - last_report >= 2.0:
                print(f"target_motion_status=moving pose={args.center_x:.2f},{y:.2f},{z:.2f}")
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

    set_pose(args.world, args.model, args.center_x, args.center_y, args.center_z)
    print("target_motion_status=stopped", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
