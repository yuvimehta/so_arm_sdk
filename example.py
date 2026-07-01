"""Minimal end-to-end demo of lerobot_sdk on an SO-101 follower arm.

Usage:
    python -m lerobot_sdk.example --port /dev/ttyACM0 --id my_arm

Run with --dry-run to exercise the imports and pose math without hardware.
"""

from __future__ import annotations

import argparse
import time

from so_arm_sdk import LeRobotArm, TCPPose


def main() -> None:
    parser = argparse.ArgumentParser(description="lerobot_sdk demo")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port of the bus-servo adapter")
    parser.add_argument("--id", dest="robot_id", default="my_arm", help="Calibration id")
    parser.add_argument("--type", dest="robot_type", default="so101_follower")
    parser.add_argument("--dry-run", action="store_true", help="Skip hardware; only test pose math")
    args = parser.parse_args()

    if args.dry_run:
        pose = TCPPose.from_position_euler([0.2, 0.0, 0.15], [0, 90, 0], degrees=True)
        print("Pose:", pose)
        print("  quat:", pose.as_quat())
        print("  euler(deg):", pose.as_euler(degrees=True))
        print("  matrix:\n", pose.to_matrix())
        roundtrip = TCPPose.from_matrix(pose.to_matrix())
        print("  roundtrip:", roundtrip)
        return

    arm = LeRobotArm(port=args.port, robot_id=args.robot_id, robot_type=args.robot_type)
    with arm:
        print("Connected:", arm.is_connected)

        joints = arm.get_joints()
        print("Joints:", {k: round(v, 2) for k, v in joints.items()})

        try:
            tcp = arm.get_tcp()
            print("TCP pose:", tcp)
        except Exception as e:  # placo / urdf may be missing
            print("TCP unavailable:", e)

        # Save the current configuration as 'home'.
        arm.save_pose("home")
        print("Saved poses:", arm.list_poses())

        # Small relative joint nudge then return home.
        nudged = dict(arm.get_joints(include_gripper=False))
        nudged["shoulder_pan"] += 10.0
        print("Moving shoulder_pan +10 deg...")
        arm.move_to_joints(nudged)
        time.sleep(0.5)

        print("Returning home...")
        arm.move_to_saved_pose("home")

        # Cartesian nudge (+2 cm in z), if kinematics available.
        try:
            tcp = arm.get_tcp()
            target = TCPPose.from_position_rotvec(
                [tcp.x, tcp.y, tcp.z + 0.02], tcp.rotvec
            )
            print("Cartesian move +2cm z...")
            arm.move_to_pose(target)
        except Exception as e:
            print("Skipping Cartesian move:", e)


if __name__ == "__main__":
    main()
