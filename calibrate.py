"""Run the calibration flow for the arm and store it under ``robot_id``.

Calibration records each joint's homing offset (middle of range) and min/max
travel so that joint angles are meaningful (0 deg = middle of range).

Usage:
    python -m lerobot_sdk.calibrate --port /dev/ttyACM0 --id my_arm

You'll be asked to:
  1. Move the arm to the middle of its range of motion, then press ENTER.
  2. Sweep each joint through its full range, then press ENTER.

The result is saved to <calibration_dir>/<id>.json and reused on every connect.
"""

from __future__ import annotations

import argparse

from so_arm_sdk import LeRobotArm


def main() -> None:
    parser = argparse.ArgumentParser(description="Calibrate the arm.")
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--id", dest="robot_id", default="so101_arm", help="Calibration id")
    parser.add_argument("--type", dest="robot_type", default="so101_follower")
    args = parser.parse_args()

    arm = LeRobotArm(port=args.port, robot_id=args.robot_id, robot_type=args.robot_type)

    # Connect without auto-calibration, then force a fresh calibration run.
    arm.connect(calibrate=False)
    try:
        arm.calibrate()
        print(f"\nCalibration complete and saved for id='{args.robot_id}'.")
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
