"""Test script: move the arm to its calibration pose (all joint angles = 0 deg).

For a calibrated arm, every joint at 0 deg is the middle of its recorded range,
i.e. the canonical "home" pose. This is a good first motion to confirm wiring,
motor IDs, and calibration are all correct.

Usage:
    python -m lerobot_sdk.go_to_zero --port /dev/ttyACM0 --id my_arm

Safety:
    The arm WILL move when this runs. Keep the workspace clear and a hand near
    the power switch. Use --speed to slow the approach.
"""

from __future__ import annotations

import argparse
import time

from lerobot_sdk import LeRobotArm


def main() -> None:
    parser = argparse.ArgumentParser(description="Move the arm to its all-zero calibration pose.")
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--id", dest="robot_id", default="so101_arm", help="Calibration id")
    parser.add_argument("--type", dest="robot_type", default="so101_follower")
    parser.add_argument(
        "--no-gripper", action="store_true", help="Leave the gripper where it is"
    )
    parser.add_argument(
        "--max-step",
        type=float,
        default=None,
        help="Optional per-step joint clamp (deg) for a gentler approach, e.g. 5",
    )
    parser.add_argument("--timeout", type=float, default=15.0, help="Arrival timeout (s)")
    args = parser.parse_args()

    arm = LeRobotArm(
        port=args.port,
        robot_id=args.robot_id,
        robot_type=args.robot_type,
        max_relative_target=args.max_step,
    )

    with arm:
        print(f"Connected to {args.robot_type} on {args.port} (id={args.robot_id}).")

        start = arm.get_joints()
        print("Current joints:", {k: round(v, 1) for k, v in start.items()})

        input("\nArm will move to the ALL-ZERO calibration pose. Press ENTER to continue (Ctrl-C to abort)...")

        print("Moving to zero...")
        if args.max_step is not None:
            # With a per-step clamp, repeatedly re-issue the target until reached.
            _approach_zero(arm, include_gripper=not args.no_gripper, timeout_s=args.timeout)
        else:
            arm.move_to_zero(include_gripper=not args.no_gripper, timeout_s=args.timeout)

        final = arm.get_joints()
        print("Final joints:", {k: round(v, 1) for k, v in final.items()})
        print("Done. Arm is at the calibration pose.")


def _approach_zero(arm: LeRobotArm, *, include_gripper: bool, timeout_s: float) -> None:
    deadline = time.perf_counter() + timeout_s
    while True:
        arm.move_to_zero(include_gripper=include_gripper, wait=False)
        current = arm.get_joints(include_gripper=include_gripper)
        if all(abs(v) <= 2.0 for k, v in current.items() if k != "gripper") and (
            not include_gripper or abs(current.get("gripper", 0.0)) <= 5.0
        ):
            return
        if time.perf_counter() >= deadline:
            print("Warning: timed out before fully reaching zero (clamped approach).")
            return
        time.sleep(0.05)


if __name__ == "__main__":
    main()
