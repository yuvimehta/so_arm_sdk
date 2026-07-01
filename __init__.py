"""lerobot_sdk - a small, task-oriented SDK for the LeRobot SO-ARM100/SO-101 arm.

Self-contained: the subset of the official ``lerobot`` codebase needed to drive
the arm (Feetech STS3215 bus servos + Waveshare bus-servo adapter) and to solve
forward/inverse kinematics is vendored under
:mod:`lerobot_sdk._vendor.lerobot_core`, so the SDK has no dependency on an
external ``lerobot`` installation or source tree. It exposes high-level helpers
for reading and commanding the arm in joint and Cartesian space, plus a
persistent named-pose library.

Example:
    >>> from lerobot_sdk import LeRobotArm
    >>> with LeRobotArm(port="/dev/ttyACM0", robot_id="my_arm") as arm:
    ...     print(arm.get_joints())
    ...     print(arm.get_tcp())
    ...     arm.save_pose("home")
    ...     arm.move_to_joints({"shoulder_pan": 10.0})
"""

from __future__ import annotations

from .arm import ALL_JOINTS, ARM_JOINTS, GRIPPER_JOINT, LeRobotArm
from .exceptions import (
    KinematicsUnavailableError,
    LeRobotSDKError,
    MotionTimeoutError,
    NotConnectedError,
    PoseNotFoundError,
)
from .pose_store import PoseStore, SavedPose
from .poses import TCPPose


def __getattr__(name: str):
    """Lazily expose the vendored robot classes.

    ``SOFollower``/``SOFollowerRobotConfig`` are imported on demand (they pull in
    optional hardware dependencies like ``pyserial``/``scservo_sdk``) so that
    merely importing ``lerobot_sdk`` stays lightweight. This also lets external
    consumers (e.g. the MoveIt arm server) use the same vendored robot the SDK
    uses, without reaching into the private ``_vendor`` namespace.
    """
    if name in ("SOFollower", "SOFollowerRobotConfig"):
        from ._vendor.lerobot_core.robots.so_follower import (
            SOFollower,
            SOFollowerRobotConfig,
        )

        return {"SOFollower": SOFollower, "SOFollowerRobotConfig": SOFollowerRobotConfig}[name]
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__ = [
    "LeRobotArm",
    "TCPPose",
    "PoseStore",
    "SavedPose",
    "ARM_JOINTS",
    "GRIPPER_JOINT",
    "ALL_JOINTS",
    "SOFollower",
    "SOFollowerRobotConfig",
    "LeRobotSDKError",
    "NotConnectedError",
    "KinematicsUnavailableError",
    "PoseNotFoundError",
    "MotionTimeoutError",
]

__version__ = "0.1.0"
