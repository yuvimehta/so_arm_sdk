"""High-level SDK for the LeRobot SO-ARM100 / SO-101 follower arm.

This wraps the vendored :class:`SOFollower` robot (Feetech STS3215 bus servos
driven through the Waveshare bus-servo adapter) and the ``RobotKinematics``
solver bundled under :mod:`lerobot_sdk._vendor.lerobot_core`, to expose a small,
task-oriented API:

* :meth:`LeRobotArm.get_joints` - read current joint angles
* :meth:`LeRobotArm.get_tcp` - read current Cartesian tool-center-point pose
* :meth:`LeRobotArm.move_to_joints` - move in joint space
* :meth:`LeRobotArm.move_to_pose` - move in Cartesian space (via inverse kinematics)
* :meth:`LeRobotArm.save_pose` / :meth:`LeRobotArm.move_to_saved_pose` - named pose library
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Sequence

import numpy as np

from .exceptions import (
    KinematicsUnavailableError,
    MotionTimeoutError,
    NotConnectedError,
)
from .pose_store import PoseStore, SavedPose
from .poses import TCPPose

# Joints in the order used by the SO follower bus / kinematic chain.
ARM_JOINTS: list[str] = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]
GRIPPER_JOINT = "gripper"
ALL_JOINTS: list[str] = [*ARM_JOINTS, GRIPPER_JOINT]

_PACKAGE_ROOT = Path(__file__).resolve().parent
_DEFAULT_URDF = _PACKAGE_ROOT / "assets" / "SO101" / "so101_new_calib.urdf"
_DEFAULT_POSE_LIBRARY = Path.home() / ".lerobot_sdk" / "poses.json"


class LeRobotArm:
    """Convenience wrapper around an SO-100/SO-101 follower arm.

    Args:
        port: Serial port of the bus-servo adapter, e.g. ``"/dev/ttyACM0"``.
        robot_id: Identifier used to locate the calibration file written by
            the calibration flow (``<calibration_dir>/<robot_id>.json``).
        robot_type: ``"so101_follower"`` (default) or ``"so100_follower"``.
        urdf_path: Path to the arm URDF used for kinematics. Defaults to the
            SO-101 URDF shipped in this repo.
        target_frame_name: End-effector frame in the URDF used as the TCP.
        use_degrees: If ``True`` joint values are in degrees (recommended).
        max_relative_target: Optional safety clamp on per-step joint motion.
        calibration_dir: Optional directory holding the calibration JSON.
        pose_library_path: Where named poses are persisted.
    """

    def __init__(
        self,
        port: str,
        *,
        robot_id: str = "so101_arm",
        robot_type: str = "so101_follower",
        urdf_path: str | Path | None = _DEFAULT_URDF,
        target_frame_name: str = "gripper_frame_link",
        use_degrees: bool = True,
        max_relative_target: float | dict[str, float] | None = None,
        calibration_dir: str | Path | None = None,
        pose_library_path: str | Path = _DEFAULT_POSE_LIBRARY,
    ):
        # Imported lazily so importing the SDK never hard-fails if an optional
        # runtime dependency (e.g. pyserial/scservo_sdk) is missing.
        from ._vendor.lerobot_core.robots.so_follower import SOFollower
        from ._vendor.lerobot_core.robots.so_follower.config_so_follower import SOFollowerRobotConfig

        self.robot_id = robot_id
        self.robot_type = robot_type
        self.use_degrees = use_degrees
        self.urdf_path = Path(urdf_path) if urdf_path is not None else None
        self.target_frame_name = target_frame_name

        config = SOFollowerRobotConfig(
            id=robot_id,
            calibration_dir=Path(calibration_dir) if calibration_dir else None,
            port=port,
            max_relative_target=max_relative_target,
            use_degrees=use_degrees,
        )
        self.robot: SOFollower = SOFollower(config)

        self.poses = PoseStore(pose_library_path)
        self._kinematics = None  # lazily constructed RobotKinematics

    # ------------------------------------------------------------------
    # Connection lifecycle
    # ------------------------------------------------------------------
    def connect(self, calibrate: bool = True) -> "LeRobotArm":
        """Open the serial connection (and calibrate if needed)."""
        self.robot.connect(calibrate=calibrate)
        return self

    def disconnect(self) -> None:
        """Close the serial connection."""
        if self.robot.is_connected:
            self.robot.disconnect()

    @property
    def is_connected(self) -> bool:
        return self.robot.is_connected

    def __enter__(self) -> "LeRobotArm":
        return self.connect()

    def __exit__(self, exc_type, exc, tb) -> None:
        self.disconnect()

    def _require_connected(self) -> None:
        if not self.robot.is_connected:
            raise NotConnectedError(
                "Arm is not connected. Call .connect() or use the arm as a context manager."
            )

    # ------------------------------------------------------------------
    # One-time setup helpers (motor IDs + calibration)
    # ------------------------------------------------------------------
    def setup_motors(self) -> None:
        """Interactively assign IDs/baudrate to each motor, one at a time.

        Run this once on a brand-new arm (or after replacing a servo). Connect
        the controller board to a single motor when prompted; the SDK writes the
        correct ID for that joint. Do NOT call :meth:`connect` first.
        """
        self.robot.setup_motors()

    @property
    def is_calibrated(self) -> bool:
        return self.robot.is_calibrated

    def calibrate(self) -> None:
        """Run the standard lerobot calibration flow for this arm.

        The arm must be connected. Calibration is also triggered automatically by
        :meth:`connect` when no calibration file is found for ``robot_id``.
        """
        self._require_connected()
        self.robot.calibrate()

    # ------------------------------------------------------------------
    # Kinematics
    # ------------------------------------------------------------------
    @property
    def kinematics(self):
        """Lazily-built :class:`RobotKinematics` solver (requires ``placo`` + URDF)."""
        if self._kinematics is None:
            self._kinematics = self._build_kinematics()
        return self._kinematics

    def _build_kinematics(self):
        if self.urdf_path is None:
            raise KinematicsUnavailableError("No URDF path was provided for kinematics.")
        if not self.urdf_path.is_file():
            raise KinematicsUnavailableError(f"URDF file not found: {self.urdf_path}")
        try:
            from ._vendor.lerobot_core.model.kinematics import RobotKinematics
        except ImportError as e:  # pragma: no cover
            raise KinematicsUnavailableError(str(e)) from e

        try:
            return RobotKinematics(
                urdf_path=str(self.urdf_path),
                target_frame_name=self.target_frame_name,
                joint_names=ARM_JOINTS,
            )
        except ImportError as e:
            raise KinematicsUnavailableError(
                "placo is required for kinematics. Install it with: pip install placo"
            ) from e

    # ------------------------------------------------------------------
    # State readers
    # ------------------------------------------------------------------
    def get_joints(self, include_gripper: bool = True) -> dict[str, float]:
        """Return the current joint positions.

        Arm joints are in degrees (when ``use_degrees=True``); the gripper is a
        0-100 normalised opening percentage.
        """
        self._require_connected()
        positions = self.robot.bus.sync_read("Present_Position")
        joints = ARM_JOINTS + ([GRIPPER_JOINT] if include_gripper else [])
        return {name: float(positions[name]) for name in joints if name in positions}

    def get_joint_array(self) -> np.ndarray:
        """Return the 5 arm-joint angles as an ordered NumPy array (degrees)."""
        joints = self.get_joints(include_gripper=False)
        return np.array([joints[name] for name in ARM_JOINTS], dtype=float)

    def get_tcp(self) -> TCPPose:
        """Return the current end-effector (tool-center-point) pose via forward kinematics."""
        q = self.get_joint_array()
        t = self.kinematics.forward_kinematics(q)
        return TCPPose.from_matrix(t)

    # Convenient aliases
    get_pose = get_tcp

    # ------------------------------------------------------------------
    # Joint-space motion
    # ------------------------------------------------------------------
    def move_to_joints(
        self,
        targets: dict[str, float] | Sequence[float],
        *,
        wait: bool = True,
        tolerance_deg: float = 2.0,
        gripper_tolerance: float = 5.0,
        timeout_s: float = 10.0,
        poll_interval_s: float = 0.02,
    ) -> dict[str, float]:
        """Command a joint-space target.

        Args:
            targets: Either a mapping ``{joint_name: value}`` (partial allowed) or
                an ordered sequence of 5 (arm) or 6 (arm + gripper) values.
            wait: Block until the arm reaches the target (or ``timeout_s``).
            tolerance_deg: Per-joint arrival tolerance for arm joints, in degrees.
            gripper_tolerance: Arrival tolerance for the gripper (0-100 units).
            timeout_s: Maximum time to wait when ``wait=True``.
            poll_interval_s: Polling period while waiting.

        Returns:
            The joint target dictionary that was actually sent (possibly clipped
            by ``max_relative_target``).
        """
        self._require_connected()
        goal = self._normalize_joint_targets(targets)
        action = {f"{name}.pos": float(value) for name, value in goal.items()}
        sent = self.robot.send_action(action)
        sent_goal = {k.removesuffix(".pos"): v for k, v in sent.items()}

        if wait:
            self._wait_until_reached(
                goal,
                tolerance_deg=tolerance_deg,
                gripper_tolerance=gripper_tolerance,
                timeout_s=timeout_s,
                poll_interval_s=poll_interval_s,
            )
        return sent_goal

    def set_gripper(self, opening: float, **kwargs) -> dict[str, float]:
        """Set only the gripper opening (0 = closed, 100 = open)."""
        return self.move_to_joints({GRIPPER_JOINT: opening}, **kwargs)

    def move_to_zero(self, *, include_gripper: bool = True, **kwargs) -> dict[str, float]:
        """Move every arm joint to 0 deg (the calibration / home pose).

        With a calibrated arm, 0 deg corresponds to the middle of each joint's
        range recorded during calibration. The gripper (if included) is moved to
        0 (fully closed).
        """
        target = {name: 0.0 for name in ARM_JOINTS}
        if include_gripper:
            target[GRIPPER_JOINT] = 0.0
        return self.move_to_joints(target, **kwargs)

    # ------------------------------------------------------------------
    # Cartesian-space motion
    # ------------------------------------------------------------------
    def move_to_pose(
        self,
        pose: TCPPose | Sequence[float] | np.ndarray,
        *,
        gripper: float | None = None,
        position_weight: float = 1.0,
        orientation_weight: float = 1.0,
        wait: bool = True,
        **wait_kwargs,
    ) -> dict[str, float]:
        """Move the end-effector to a Cartesian pose using inverse kinematics.

        Args:
            pose: A :class:`TCPPose`, a 4x4 transform, or ``[x, y, z, rx, ry, rz]``
                (position in metres, orientation as a rotation vector in radians).
            gripper: Optional gripper opening (0-100). If ``None`` it is unchanged.
            position_weight: IK weight on position error.
            orientation_weight: IK weight on orientation error (set 0 for
                position-only).
            wait: Block until the arm arrives.

        Returns:
            The joint target dictionary that was sent.
        """
        self._require_connected()
        target_matrix = self._coerce_pose(pose).to_matrix()

        q_current = self.get_joint_array()
        q_target = self.kinematics.inverse_kinematics(
            q_current,
            target_matrix,
            position_weight=position_weight,
            orientation_weight=orientation_weight,
        )

        goal = {name: float(q_target[i]) for i, name in enumerate(ARM_JOINTS)}
        if gripper is not None:
            goal[GRIPPER_JOINT] = float(gripper)

        return self.move_to_joints(goal, wait=wait, **wait_kwargs)

    def solve_ik(
        self,
        pose: TCPPose | Sequence[float] | np.ndarray,
        *,
        position_weight: float = 1.0,
        orientation_weight: float = 1.0,
    ) -> dict[str, float]:
        """Solve inverse kinematics for ``pose`` without moving the arm."""
        self._require_connected()
        q_current = self.get_joint_array()
        target_matrix = self._coerce_pose(pose).to_matrix()
        q_target = self.kinematics.inverse_kinematics(
            q_current, target_matrix, position_weight=position_weight, orientation_weight=orientation_weight
        )
        return {name: float(q_target[i]) for i, name in enumerate(ARM_JOINTS)}

    # ------------------------------------------------------------------
    # Pose library
    # ------------------------------------------------------------------
    def save_pose(self, name: str, *, with_tcp: bool = True) -> SavedPose:
        """Capture the current joint configuration (and TCP) under ``name``.

        The TCP is only stored when kinematics are available; failures there are
        non-fatal so a pose can always be saved in joint space.
        """
        self._require_connected()
        joints = self.get_joints(include_gripper=True)

        tcp_dict = None
        if with_tcp:
            try:
                tcp_dict = self.get_tcp().to_dict()
            except KinematicsUnavailableError:
                tcp_dict = None

        pose = SavedPose(name=name, joints=joints, tcp=tcp_dict, robot_id=self.robot_id)
        return self.poses.save(pose)

    def list_poses(self) -> list[str]:
        """Names of all saved poses."""
        return self.poses.names()

    def get_saved_pose(self, name: str) -> SavedPose:
        """Return a saved pose record by name."""
        return self.poses.get(name)

    def delete_pose(self, name: str) -> None:
        """Remove a saved pose."""
        self.poses.delete(name)

    def move_to_saved_pose(self, name: str, **kwargs) -> dict[str, float]:
        """Replay a saved pose in joint space (exact, no IK)."""
        self._require_connected()
        saved = self.poses.get(name)
        return self.move_to_joints(saved.joints, **kwargs)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _normalize_joint_targets(
        self, targets: dict[str, float] | Sequence[float]
    ) -> dict[str, float]:
        if isinstance(targets, dict):
            unknown = set(targets) - set(ALL_JOINTS)
            if unknown:
                raise ValueError(f"Unknown joint name(s): {sorted(unknown)}")
            return {name: float(value) for name, value in targets.items()}

        values = list(targets)
        if len(values) == len(ARM_JOINTS):
            names = ARM_JOINTS
        elif len(values) == len(ALL_JOINTS):
            names = ALL_JOINTS
        else:
            raise ValueError(
                f"Expected {len(ARM_JOINTS)} or {len(ALL_JOINTS)} joint values, got {len(values)}"
            )
        return {name: float(value) for name, value in zip(names, values)}

    @staticmethod
    def _coerce_pose(pose: TCPPose | Sequence[float] | np.ndarray) -> TCPPose:
        if isinstance(pose, TCPPose):
            return pose
        arr = np.asarray(pose, dtype=float)
        if arr.shape == (4, 4):
            return TCPPose.from_matrix(arr)
        if arr.shape == (6,):
            return TCPPose.from_list(arr)
        raise ValueError(
            "pose must be a TCPPose, a 4x4 transform, or [x, y, z, rx, ry, rz]"
        )

    def _wait_until_reached(
        self,
        goal: dict[str, float],
        *,
        tolerance_deg: float,
        gripper_tolerance: float,
        timeout_s: float,
        poll_interval_s: float,
    ) -> None:
        deadline = time.perf_counter() + timeout_s
        while True:
            current = self.get_joints(include_gripper=True)
            if self._within_tolerance(goal, current, tolerance_deg, gripper_tolerance):
                return
            if time.perf_counter() >= deadline:
                errors = {
                    name: abs(goal[name] - current.get(name, goal[name])) for name in goal
                }
                worst = max(errors, key=errors.get)
                raise MotionTimeoutError(
                    f"Arm did not reach target within {timeout_s}s. "
                    f"Largest remaining error: {worst}={errors[worst]:.2f}"
                )
            time.sleep(poll_interval_s)

    @staticmethod
    def _within_tolerance(
        goal: dict[str, float],
        current: dict[str, float],
        tolerance_deg: float,
        gripper_tolerance: float,
    ) -> bool:
        for name, target in goal.items():
            if name not in current:
                continue
            tol = gripper_tolerance if name == GRIPPER_JOINT else tolerance_deg
            if abs(target - current[name]) > tol:
                return False
        return True
