"""Assign IDs (and baudrate) to the Feetech servos, one motor at a time.

Every Feetech STS3215 servo ships with the same default ID (1). Before the arm
can be used they must each get a unique ID matching their joint:

    shoulder_pan=1, shoulder_lift=2, elbow_flex=3,
    wrist_flex=4, wrist_roll=5, gripper=6

This script walks you through it: connect the Waveshare bus-servo adapter to a
SINGLE motor at a time when prompted, and it writes that motor's ID.

Usage:
    python -m lerobot_sdk.setup_motors --port /dev/ttyACM0 --id my_arm

Equivalent official CLI (does the same thing):
    lerobot-setup-motors --robot.type=so101_follower --robot.port=/dev/ttyACM0
"""

from __future__ import annotations

import argparse

from lerobot_sdk import LeRobotArm


def main() -> None:
    parser = argparse.ArgumentParser(description="Set Feetech motor IDs for the arm.")
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--id", dest="robot_id", default="so101_arm", help="Calibration id")
    parser.add_argument("--type", dest="robot_type", default="so101_follower")
    args = parser.parse_args()

    arm = LeRobotArm(port=args.port, robot_id=args.robot_id, robot_type=args.robot_type)

    print(
        "Motor ID setup.\n"
        "You'll be prompted for each joint, from the gripper back to the base.\n"
        "Connect the bus-servo adapter to ONLY the requested motor each time.\n"
    )
    # Do not connect() first: setup writes IDs to a single connected motor.
    arm.setup_motors()
    print("\nAll motor IDs assigned. Next: calibrate with `python -m lerobot_sdk.calibrate`.")


if __name__ == "__main__":
    main()
