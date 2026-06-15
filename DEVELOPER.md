# lerobot_sdk — Developer Documentation

This document explains the internals of `lerobot_sdk`: the architecture, how the
modules fit together, and — for every script in the package — the logic and the
specific code that makes it work.

It is meant for contributors/maintainers. For usage instructions see
[`README.md`](./README.md).

---

## 1. Architecture overview

`lerobot_sdk` is a **thin, task-oriented wrapper** over the official `lerobot`
package. It does not talk to motors directly; it delegates all hardware I/O to
`lerobot`'s `SOFollower` robot and `FeetechMotorsBus`, and all kinematics to
`lerobot`'s `RobotKinematics` (placo). The SDK's job is to provide an ergonomic
surface (`get_joints`, `get_tcp`, `move_to_joints`, `move_to_pose`, pose
library) plus a few CLI tools.

```
            ┌─────────────────────────────────────────────────────────┐
  CLI tools │ setup_motors  calibrate  go_to_zero  joint_controller    │
            │ joint_gui     example                                    │
            └───────────────────────────┬─────────────────────────────┘
                                         │  (all use)
                            ┌────────────▼────────────┐
            SDK core        │   LeRobotArm  (arm.py)   │
                            │  + TCPPose   (poses.py)  │
                            │  + PoseStore (pose_store)│
                            └────────────┬────────────┘
                                         │  (delegates to)
            ┌────────────────────────────▼────────────────────────────┐
  lerobot   │ SOFollower → FeetechMotorsBus → scservo_sdk (serial)      │
  (official)│ RobotKinematics → placo → pinocchio (URDF FK/IK)          │
            └────────────────────────────┬─────────────────────────────┘
                                         │  (drives)
            ┌────────────────────────────▼────────────────────────────┐
  Hardware  │ Waveshare bus-servo adapter → 6× Feetech STS3215 servos   │
            └──────────────────────────────────────────────────────────┘
```

### Package layout

| File | Role |
| --- | --- |
| `__init__.py` | Package exports + the `lerobot` import bootstrap |
| `exceptions.py` | SDK-specific exception hierarchy |
| `poses.py` | `TCPPose` dataclass + pure-NumPy rotation conversions |
| `pose_store.py` | `SavedPose` + `PoseStore` (JSON-backed named pose library) |
| `arm.py` | `LeRobotArm` — the core API |
| `setup_motors.py` | CLI: assign motor IDs (once per arm) |
| `calibrate.py` | CLI: run calibration (once per arm) |
| `go_to_zero.py` | CLI/test: move to the all-zero calibration pose |
| `joint_controller.py` | CLI: keyboard joint control (live + REPL) |
| `joint_gui.py` | CLI: Tkinter slider GUI |
| `example.py` | End-to-end demo (+ `--dry-run` pose math) |
| `requirements.txt` | Dependencies |

### Conventions used across the SDK
- **Joint order** is fixed and matches the SO follower bus / kinematic chain:
  `shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper`.
- **Units:** arm joints in degrees, gripper as a 0–100 opening, TCP position in
  metres, TCP orientation as a rotation vector (axis-angle, radians).
- **Kinematics use the 5 arm joints only** (the gripper does not move the TCP
  frame); the gripper is handled separately.

---

## 2. `__init__.py` — exports and the import bootstrap

The most important logic here is `_bootstrap_lerobot()`, which guarantees the
SDK uses **this repo's** `lerobot/src` even if a different (often older)
`lerobot` is pip-installed in the environment.

```24:53:lerobot_sdk/__init__.py
def _bootstrap_lerobot() -> None:
    """Ensure the in-repo ``lerobot`` source is the one that gets imported.

    This repo ships the official ``lerobot`` codebase under ``lerobot/src``. Some
    environments also have a different (often older) ``lerobot`` installed via
    pip that lacks modules like ``lerobot.robots``. To guarantee the SDK uses
    *this* repo's code, we put the in-repo source at the front of ``sys.path``
    and purge any already-imported, mismatched ``lerobot`` modules.
    """
    src = _Path(__file__).resolve().parent.parent / "lerobot" / "src"
    if not (src / "lerobot" / "robots" / "__init__.py").is_file():
        # No in-repo source available; fall back to whatever is installed.
        return

    src_str = str(src)
    # Give the in-repo source top priority on the import path.
    if src_str in _sys.path:
        _sys.path.remove(src_str)
    _sys.path.insert(0, src_str)

    # If a mismatched lerobot was already imported, drop it so the in-repo one
    # is loaded fresh from src.
    existing = _sys.modules.get("lerobot")
    if existing is not None:
        mod_file = getattr(existing, "__file__", "") or ""
        if not mod_file.replace("\\", "/").startswith(src_str.replace("\\", "/")):
            for _name in [
                _n for _n in list(_sys.modules) if _n == "lerobot" or _n.startswith("lerobot.")
            ]:
                del _sys.modules[_name]
```

**Why each step matters:**
- It checks for `lerobot/src/lerobot/robots/__init__.py` specifically (not just
  `lerobot`), because the failure mode we're guarding against is a *partial*
  install that has `lerobot` but not `lerobot.robots`.
- `sys.path.insert(0, ...)` ensures the repo source wins over site-packages.
- The **purge loop** handles the case where something already imported the wrong
  `lerobot` earlier in the process: Python caches the parent package's
  `__path__`, so simply changing `sys.path` wouldn't redirect submodule imports.
  Deleting the cached modules forces a fresh import from `src`.

The bootstrap is called at import time, **before** any `from lerobot...` import:

```56:67:lerobot_sdk/__init__.py
_bootstrap_lerobot()

from .arm import ALL_JOINTS, ARM_JOINTS, GRIPPER_JOINT, LeRobotArm  # noqa: E402
from .exceptions import (  # noqa: E402
    KinematicsUnavailableError,
    LeRobotSDKError,
    MotionTimeoutError,
    NotConnectedError,
    PoseNotFoundError,
)
from .pose_store import PoseStore, SavedPose  # noqa: E402
from .poses import TCPPose  # noqa: E402
```

> Note: `arm.py` itself imports `lerobot` **lazily** inside `LeRobotArm.__init__`,
> not at module top-level, so importing `lerobot_sdk` never hard-fails even if
> `lerobot`/`placo` aren't installed — only constructing a `LeRobotArm` does.

---

## 3. `exceptions.py` — error hierarchy

A small, flat hierarchy rooted at `LeRobotSDKError`, so callers can catch
everything from the SDK with one `except`. Two classes intentionally multiply
inherit from built-ins so they also satisfy idiomatic `except` clauses:

```20:25:lerobot_sdk/exceptions.py
class PoseNotFoundError(LeRobotSDKError, KeyError):
    """Raised when a named pose is not present in the pose library."""


class MotionTimeoutError(LeRobotSDKError, TimeoutError):
    """Raised when a blocking move does not reach its target within the timeout."""
```

- `PoseNotFoundError` is also a `KeyError` (pose lookups feel like dict lookups).
- `MotionTimeoutError` is also a `TimeoutError`.
- `KinematicsUnavailableError` signals a missing `placo` or URDF, used to make
  TCP features degrade gracefully (see `save_pose`).

---

## 4. `poses.py` — `TCPPose` and rotation math

This module is deliberately **dependency-light** (only NumPy) so it can be used
for serialisation/bookkeeping without `placo`. It has two parts.

### 4.1 Rotation conversion helpers

Pure functions converting between rotation representations. Example —
axis-angle (rotvec) → rotation matrix via Rodrigues' formula, with a
small-angle guard:

```21:38:lerobot_sdk/poses.py
def matrix_from_rotvec(rotvec: np.ndarray) -> np.ndarray:
    """Convert an axis-angle rotation vector to a 3x3 rotation matrix."""
    rotvec = np.asarray(rotvec, dtype=float).reshape(3)
    theta = float(np.linalg.norm(rotvec))
    if theta < 1e-12:
        return np.eye(3)
    axis = rotvec / theta
    x, y, z = axis
    c = np.cos(theta)
    s = np.sin(theta)
    c1 = 1.0 - c
    return np.array(
        [
            [c + x * x * c1, x * y * c1 - z * s, x * z * c1 + y * s],
            [y * x * c1 + z * s, c + y * y * c1, y * z * c1 - x * s],
            [z * x * c1 - y * s, z * y * c1 + x * s, c + z * z * c1],
        ]
    )
```

The inverse (`rotvec_from_matrix`) handles the numerically tricky **180° case**
separately, where `sin(angle) ≈ 0` and the simple antisymmetric-part formula
breaks down:

```47:58:lerobot_sdk/poses.py
    if abs(angle - np.pi) < 1e-6:
        # Near 180 deg: use the most numerically stable column.
        a = (m + np.eye(3)) / 2.0
        axis = np.sqrt(np.clip(np.diag(a), 0.0, None))
        # Recover signs from off-diagonal terms.
        if axis[0] > 1e-6:
            axis[1] = np.copysign(axis[1], a[0, 1])
            axis[2] = np.copysign(axis[2], a[0, 2])
        elif axis[1] > 1e-6:
            axis[2] = np.copysign(axis[2], a[1, 2])
        axis = axis / np.linalg.norm(axis)
        return axis * angle
```

There are matching `quat_from_matrix`/`matrix_from_quat` (the quaternion path
uses the standard trace-based branch selection for numerical stability) and
`matrix_from_euler`/`euler_from_matrix` (intrinsic XYZ).

### 4.2 The `TCPPose` dataclass

Stores position (`x,y,z`, metres) and orientation as a rotvec (`rx,ry,rz`,
radians). Internally everything funnels through a 4×4 homogeneous matrix, which
is the lingua franca with `RobotKinematics`.

```165:184:lerobot_sdk/poses.py
    @classmethod
    def from_matrix(cls, t: np.ndarray) -> "TCPPose":
        t = np.asarray(t, dtype=float)
        pos = t[:3, 3]
        rotvec = rotvec_from_matrix(t[:3, :3])
        return cls(float(pos[0]), float(pos[1]), float(pos[2]), *map(float, rotvec))

    @classmethod
    def from_position_rotvec(cls, position, rotvec) -> "TCPPose":
        p = np.asarray(position, dtype=float).reshape(3)
        r = np.asarray(rotvec, dtype=float).reshape(3)
        return cls(*map(float, p), *map(float, r))

    @classmethod
    def from_position_quat(cls, position, quat) -> "TCPPose":
        return cls.from_matrix(_compose(position, matrix_from_quat(quat)))

    @classmethod
    def from_position_euler(cls, position, euler, *, degrees: bool = False) -> "TCPPose":
        return cls.from_matrix(_compose(position, matrix_from_euler(euler, degrees=degrees)))
```

`to_matrix()` rebuilds the 4×4 (used when handing a target to IK), and
`as_quat()` / `as_euler()` provide alternate views. The `_compose` helper just
packs a rotation matrix + position into a 4×4:

```244:248:lerobot_sdk/poses.py
def _compose(position, rotation_matrix: np.ndarray) -> np.ndarray:
    t = np.eye(4, dtype=float)
    t[:3, :3] = np.asarray(rotation_matrix, dtype=float)
    t[:3, 3] = np.asarray(position, dtype=float).reshape(3)
    return t
```

`to_dict()`/`from_dict()` provide JSON-friendly (de)serialisation used by the
pose store.

---

## 5. `pose_store.py` — the named pose library

### `SavedPose`
A dataclass capturing **both** the joint configuration and (optionally) the TCP
pose for one named waypoint. Joints are the source of truth for exact replay;
the TCP is for inspection/Cartesian replay.

```19:31:lerobot_sdk/pose_store.py
@dataclass
class SavedPose:
    """A single named waypoint."""

    name: str
    joints: dict[str, float]
    tcp: dict[str, float] | None = None
    robot_id: str | None = None
    created_at: float = field(default_factory=time.time)

    @property
    def tcp_pose(self) -> TCPPose | None:
        return TCPPose.from_dict(self.tcp) if self.tcp else None
```

### `PoseStore`
Loads the whole JSON file into memory on construction, and writes back on every
mutation. The key robustness detail is the **atomic write** — it writes to a
temp file and `replace()`s, so an interrupted save can't corrupt your library:

```64:70:lerobot_sdk/pose_store.py
    def _flush(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        serialisable = {name: pose.to_dict() for name, pose in self._poses.items()}
        tmp = self.path.with_suffix(self.path.suffix + ".tmp")
        with open(tmp, "w") as f:
            json.dump(serialisable, f, indent=2)
        tmp.replace(self.path)
```

`get`/`delete` raise `PoseNotFoundError` for missing names; `names()`, `all()`,
`__contains__`, and `__len__` round out the dict-like access.

---

## 6. `arm.py` — `LeRobotArm` (the core)

This is where everything comes together. Module-level constants pin the joint
order and default file locations:

```30:45:lerobot_sdk/arm.py
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

_REPO_ROOT = Path(__file__).resolve().parent.parent
_DEFAULT_URDF = (
    _REPO_ROOT / "lerobot_integration" / "SO-ARM100" / "Simulation" / "SO101" / "so101_new_calib.urdf"
)
_DEFAULT_POSE_LIBRARY = Path.home() / ".lerobot_sdk" / "poses.json"
```

### 6.1 Construction

`__init__` builds the official `SOFollowerRobotConfig` + `SOFollower` and the
pose store. The `lerobot` imports are **lazy** (inside the method) so importing
the SDK never requires `lerobot`:

```78:99:lerobot_sdk/arm.py
        # Imported lazily so that importing the SDK never hard-fails if lerobot
        # is missing; the bootstrap in __init__.py adds the in-repo source tree.
        from lerobot.robots.so_follower import SOFollower
        from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig

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
```

Key choices: `robot_id` maps to the lerobot calibration filename;
`max_relative_target` is forwarded as a hardware-level per-step safety clamp;
`SO100Follower`/`SO101Follower` are the same `SOFollower` class so `robot_type`
selects the registered config.

### 6.2 Connection lifecycle

Thin pass-throughs to the underlying robot, plus a context-manager and a
`_require_connected()` guard used by every hardware-touching method:

```104:128:lerobot_sdk/arm.py
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
```

### 6.3 One-time setup (`setup_motors`, `calibrate`)

Both delegate to the official `SOFollower` implementations. Note `setup_motors`
must run **without** connecting first (it talks to a single motor):

```133:153:lerobot_sdk/arm.py
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
```

### 6.4 Kinematics (lazy)

The `RobotKinematics` solver is built **on first use** and cached, so joint-space
features never pay the placo/URDF cost. Failures are translated into
`KinematicsUnavailableError` with actionable messages. Crucially, it passes
`joint_names=ARM_JOINTS` so only the 5 arm joints participate (the gripper does
not affect the `gripper_frame_link` TCP):

```158:184:lerobot_sdk/arm.py
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
            from lerobot.model.kinematics import RobotKinematics
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
```

### 6.5 State readers (`get_joints`, `get_joint_array`, `get_tcp`)

`get_joints` reads `Present_Position` straight from the bus (avoids camera
overhead of `get_observation`) and filters to the canonical joint set:

```189:209:lerobot_sdk/arm.py
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
```

`get_tcp` = forward kinematics on the 5 arm joints → 4×4 → `TCPPose`. `get_pose`
is an alias.

### 6.6 Joint-space motion (`move_to_joints`)

This is the central command path. It normalises the target, sends it via the
robot's `send_action` (which applies `max_relative_target` clamping), and
optionally blocks until arrival:

```242:256:lerobot_sdk/arm.py
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
```

Two convenience wrappers build on it — `set_gripper` and `move_to_zero` (the
calibration/home pose; gripper 0 = closed):

```262:272:lerobot_sdk/arm.py
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
```

### 6.7 Cartesian-space motion (`move_to_pose`, `solve_ik`)

`move_to_pose` converts the requested pose to a 4×4, reads current joints as the
IK seed, solves IK for the 5 arm joints, optionally appends a gripper target,
then reuses `move_to_joints`:

```301:316:lerobot_sdk/arm.py
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
```

Note the default `orientation_weight=1.0` (the SDK honours orientation by
default; pass `0.0` for position-only). `solve_ik` does the same math but returns
the joint dict without moving — useful for previewing/validating reachability.

### 6.8 Pose library methods

`save_pose` captures joints + (best-effort) TCP. The TCP capture is wrapped so a
missing placo/URDF never blocks saving a joint-space pose:

```337:354:lerobot_sdk/arm.py
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
```

`move_to_saved_pose` replays a stored pose **in joint space** (exact, no IK):

```368:372:lerobot_sdk/arm.py
    def move_to_saved_pose(self, name: str, **kwargs) -> dict[str, float]:
        """Replay a saved pose in joint space (exact, no IK)."""
        self._require_connected()
        saved = self.poses.get(name)
        return self.move_to_joints(saved.joints, **kwargs)
```

### 6.9 Internal helpers

`_normalize_joint_targets` accepts either a (partial) dict or an ordered
sequence of 5/6 values, validating names/length:

```377:395:lerobot_sdk/arm.py
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
```

`_coerce_pose` lets `move_to_pose`/`solve_ik` accept a `TCPPose`, a 4×4 matrix,
or a `[x,y,z,rx,ry,rz]` list. `_wait_until_reached` polls `get_joints` until
within tolerance or it raises `MotionTimeoutError` reporting the worst joint:

```419:433:lerobot_sdk/arm.py
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
```

`_within_tolerance` uses a separate, looser tolerance for the gripper (0–100
units vs degrees).

---

## 7. `setup_motors.py` — assign motor IDs (CLI)

Walks the user through giving each servo its joint ID. It deliberately does
**not** call `connect()` first, then delegates to `arm.setup_motors()` (which
loops the motors and writes IDs one at a time):

```33:42:lerobot_sdk/setup_motors.py
    arm = LeRobotArm(port=args.port, robot_id=args.robot_id, robot_type=args.robot_type)

    print(
        "Motor ID setup.\n"
        "You'll be prompted for each joint, from the gripper back to the base.\n"
        "Connect the bus-servo adapter to ONLY the requested motor each time.\n"
    )
    # Do not connect() first: setup writes IDs to a single connected motor.
    arm.setup_motors()
    print("\nAll motor IDs assigned. Next: calibrate with `python -m lerobot_sdk.calibrate`.")
```

> Possible enhancement (discussed): make this resumable by persisting which
> motors were successfully assigned (keyed by `--id`) and re-doing only the
> remaining/failed ones. Not yet implemented in this file.

---

## 8. `calibrate.py` — run calibration (CLI)

Connects **without** auto-calibration and then forces a fresh calibration run,
disconnecting cleanly in a `finally`:

```30:38:lerobot_sdk/calibrate.py
    arm = LeRobotArm(port=args.port, robot_id=args.robot_id, robot_type=args.robot_type)

    # Connect without auto-calibration, then force a fresh calibration run.
    arm.connect(calibrate=False)
    try:
        arm.calibrate()
        print(f"\nCalibration complete and saved for id='{args.robot_id}'.")
    finally:
        arm.disconnect()
```

Calibration records each joint's homing offset and min/max travel; afterwards
`0°` means the middle of each joint's range. The result is stored under
`<calibration_dir>/<id>.json` and reused on every `connect`.

---

## 9. `go_to_zero.py` — move to the all-zero pose (CLI/test)

A first-motion sanity test. It connects (context manager), prints state, prompts
for confirmation, then moves to zero. With `--max-step` it switches to a
**clamped, iterative approach** because a single clamped command only moves part
way:

```55:64:lerobot_sdk/go_to_zero.py
        print("Moving to zero...")
        if args.max_step is not None:
            # With a per-step clamp, repeatedly re-issue the target until reached.
            _approach_zero(arm, include_gripper=not args.no_gripper, timeout_s=args.timeout)
        else:
            arm.move_to_zero(include_gripper=not args.no_gripper, timeout_s=args.timeout)
```

The `_approach_zero` loop re-issues the zero target (non-blocking) until all arm
joints are within 2° (and the gripper within 5 units), or it times out:

```67:79:lerobot_sdk/go_to_zero.py
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
```

Why the clamp matters: `--max-step` maps to `max_relative_target`, which caps
how far `send_action` will move per call; re-issuing repeatedly "walks" the arm
to the target in safe increments.

---

## 10. `joint_controller.py` — keyboard control (CLI)

Two control modes, chosen automatically by whether stdin is a TTY:

```215:218:lerobot_sdk/joint_controller.py
        if sys.stdin.isatty():
            controller.run_live()
        else:
            controller.run_repl()
```

### Shared state and sending
The controller keeps a `target` dict seeded from the measured position, and a
single `_send` that streams it **non-blocking** so the UI stays responsive:

```79:94:lerobot_sdk/joint_controller.py
    def _send(self) -> None:
        self.arm.move_to_joints(dict(self.target), wait=False)

    def nudge(self, delta: float) -> None:
        j = JOINTS[self.active]
        self.target[j] = _clamp(j, self.target[j] + delta)
        self._send()

    def set_joint(self, joint: str, value: float) -> None:
        self.target[joint] = _clamp(joint, value)
        self._send()

    def go_zero(self) -> None:
        for j in JOINTS:
            self.target[j] = 0.0
        self._send()
```

### Live mode (single keypress, no Enter)
It puts the terminal into cbreak mode with `tty.setcbreak`, reads one character
at a time, and **always restores** the terminal in a `finally`:

```97:115:lerobot_sdk/joint_controller.py
    def run_live(self) -> None:
        import termios
        import tty

        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        _print_help()
        print(self._status_line(), end="\r", flush=True)
        try:
            tty.setcbreak(fd)
            while True:
                ch = sys.stdin.read(1)
                if ch in ("q", "\x03"):  # q or Ctrl-C
                    break
                self._handle_key(ch)
                print(self._status_line() + "   ", end="\r", flush=True)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            print()
```

`_handle_key` maps keys to actions; note the arrow-key handling reads the rest
of the ANSI escape sequence (`ESC [ A/B`):

```136:141:lerobot_sdk/joint_controller.py
        elif ch == "\x1b":  # arrow-key escape sequence: ESC [ A/B
            seq = sys.stdin.read(2)
            if seq == "[A":
                self.nudge(self.step)
            elif seq == "[B":
                self.nudge(-self.step)
```

### REPL mode (fallback)
When stdin isn't interactive, it parses one command per line (`<joint> <value>`,
`g <value>`, `z`, `r`, `p`, `q`). `_resolve_joint` accepts either a name or a
1-based index:

```189:194:lerobot_sdk/joint_controller.py
    @staticmethod
    def _resolve_joint(token: str) -> str | None:
        if token.isdigit():
            idx = int(token) - 1
            return JOINTS[idx] if 0 <= idx < len(JOINTS) else None
        return token if token in JOINTS else None
```

---

## 11. `joint_gui.py` — Tkinter slider GUI (CLI)

A desktop GUI: one slider per joint plus telemetry. Built on stdlib `tkinter`
(imported inside the class/`main`, so importing the module never requires a
display).

### Construction & periodic loops
On launch it builds the UI, **syncs sliders to the current pose** (so the arm
doesn't jump), and starts two `after()` timers:

```50:60:lerobot_sdk/joint_gui.py
        self.root = tk.Tk()
        self.root.title(title)
        self.root.minsize(560, 460)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._build_ui()
        self._sync_sliders_to_arm()

        # Start periodic loops.
        self.root.after(SEND_PERIOD_MS, self._tick_send)
        self.root.after(TELEMETRY_PERIOD_MS, self._tick_telemetry)
```

### The "no accidental motion" mechanism
Slider callbacks set a `_dirty` flag rather than sending immediately. Two flags
coordinate this:
- `_suppress`: set while we move sliders programmatically, so those changes don't
  count as user input.
- `_dirty`: set when the user actually moves a slider.

```132:162:lerobot_sdk/joint_gui.py
    def _on_slider(self, joint: str) -> None:
        if self._suppress:
            return
        value = float(self._vars[joint].get())
        self._target[joint] = value
        unit = "" if joint == GRIPPER_JOINT else "°"
        self._value_labels[joint].config(text=f"{value:.1f}{unit}")
        self._dirty = True

    def _set_sliders(self, values: dict[str, float]) -> None:
        """Programmatically move sliders without triggering a send."""
        self._suppress = True
        try:
            for joint, val in values.items():
                if joint not in self._vars:
                    continue
                self._vars[joint].set(val)
                self._target[joint] = float(val)
                unit = "" if joint == GRIPPER_JOINT else "°"
                self._value_labels[joint].config(text=f"{float(val):.1f}{unit}")
        finally:
            self._suppress = False

    def _sync_sliders_to_arm(self) -> None:
        try:
            current = self.arm.get_joints(include_gripper=True)
        except Exception as e:
            self._set_status(f"Read failed: {e}")
            return
        self._set_sliders(current)
        self._dirty = False  # don't re-send what we just read
```

### Throttled streaming
`_tick_send` runs every `SEND_PERIOD_MS` (60 ms) and only sends if `_dirty` and
the "Send" checkbox is on — this rate-limits bus traffic during a drag and
re-arms itself:

```189:196:lerobot_sdk/joint_gui.py
    def _tick_send(self) -> None:
        if self._dirty and self.send_enabled.get():
            try:
                self.arm.move_to_joints(dict(self._target), wait=False)
                self._dirty = False
            except Exception as e:
                self._set_status(f"Send failed: {e}")
        self.root.after(SEND_PERIOD_MS, self._tick_send)
```

`_tick_telemetry` (every 400 ms) reads measured joints and TCP for the status
line, degrading to "n/a" if kinematics are unavailable. `_on_close` disconnects
before destroying the window. `main` also guards the `tkinter` import with a
friendly `apt-get install python3-tk` hint.

---

## 12. `example.py` — end-to-end demo

Has a `--dry-run` path that exercises only the pose math (no hardware), useful
for CI/smoke tests:

```25:33:lerobot_sdk/example.py
    if args.dry_run:
        pose = TCPPose.from_position_euler([0.2, 0.0, 0.15], [0, 90, 0], degrees=True)
        print("Pose:", pose)
        print("  quat:", pose.as_quat())
        print("  euler(deg):", pose.as_euler(degrees=True))
        print("  matrix:\n", pose.to_matrix())
        roundtrip = TCPPose.from_matrix(pose.to_matrix())
        print("  roundtrip:", roundtrip)
        return
```

The hardware path demonstrates the full API surface: connect → `get_joints` →
`get_tcp` → `save_pose` → `move_to_joints` → `move_to_saved_pose` →
`move_to_pose`, each wrapped so missing kinematics degrade gracefully.

---

## 13. Extending the SDK

- **Add a new high-level motion:** implement it on `LeRobotArm` in terms of
  `move_to_joints` (joint space) or `move_to_pose` (Cartesian) so you inherit the
  arrival-wait and safety clamp behaviour for free.
- **Support a different arm/URDF:** pass `robot_type=`, `urdf_path=`, and
  `target_frame_name=` to `LeRobotArm`. If the joint set differs, update
  `ARM_JOINTS`/`GRIPPER_JOINT` (kinematics use `ARM_JOINTS`).
- **Change where poses live:** pass `pose_library_path=` (per-arm libraries by
  using the `robot_id` in the path).
- **New CLI tool:** mirror the existing pattern — `argparse` for `--port/--id/
  --type`, construct `LeRobotArm`, use the context manager, keep sends
  non-blocking for interactive tools.

## 14. Validation notes

- `poses.py` is pure NumPy and round-trips (matrix ↔ rotvec ↔ quat ↔ euler);
  `example.py --dry-run` is the quickest check.
- `pose_store.py` round-trips through JSON and writes atomically.
- Anything touching hardware requires a connected, calibrated arm and (for
  TCP/Cartesian) `placo` + the URDF.
