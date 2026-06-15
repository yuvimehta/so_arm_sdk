"""Exception types raised by :mod:`lerobot_sdk`."""


class LeRobotSDKError(Exception):
    """Base class for all SDK errors."""


class NotConnectedError(LeRobotSDKError):
    """Raised when an operation requires an open connection to the arm."""


class KinematicsUnavailableError(LeRobotSDKError):
    """Raised when forward/inverse kinematics are requested but unavailable.

    This usually means the optional ``placo`` dependency or the robot URDF file
    could not be loaded.
    """


class PoseNotFoundError(LeRobotSDKError, KeyError):
    """Raised when a named pose is not present in the pose library."""


class MotionTimeoutError(LeRobotSDKError, TimeoutError):
    """Raised when a blocking move does not reach its target within the timeout."""
