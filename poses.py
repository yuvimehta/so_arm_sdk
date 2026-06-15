"""Cartesian pose representation and rotation conversions.

The :class:`TCPPose` stores the tool-center-point (end-effector) pose as a
position in metres and an orientation as a rotation vector (axis-angle, in
radians).  All conversions are implemented with plain NumPy so this module has
no dependency on the heavier kinematics stack (``placo``) and can be used purely
for bookkeeping/serialisation.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# ---------------------------------------------------------------------------
# Rotation conversion helpers (pure numpy, no external deps)
# ---------------------------------------------------------------------------


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


def rotvec_from_matrix(matrix: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to an axis-angle rotation vector."""
    m = np.asarray(matrix, dtype=float)[:3, :3]
    angle = np.arccos(np.clip((np.trace(m) - 1.0) / 2.0, -1.0, 1.0))
    if angle < 1e-12:
        return np.zeros(3)
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
    rx = m[2, 1] - m[1, 2]
    ry = m[0, 2] - m[2, 0]
    rz = m[1, 0] - m[0, 1]
    axis = np.array([rx, ry, rz]) / (2.0 * np.sin(angle))
    return axis * angle


def matrix_from_quat(quat: np.ndarray) -> np.ndarray:
    """Convert a quaternion ``[x, y, z, w]`` to a 3x3 rotation matrix."""
    x, y, z, w = np.asarray(quat, dtype=float).reshape(4)
    n = np.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def quat_from_matrix(matrix: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to a quaternion ``[x, y, z, w]``."""
    m = np.asarray(matrix, dtype=float)[:3, :3]
    t = np.trace(m)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])


def matrix_from_euler(angles: np.ndarray, *, degrees: bool = False) -> np.ndarray:
    """Convert intrinsic XYZ Euler angles (roll, pitch, yaw) to a rotation matrix."""
    a = np.asarray(angles, dtype=float).reshape(3)
    if degrees:
        a = np.deg2rad(a)
    cx, cy, cz = np.cos(a)
    sx, sy, sz = np.sin(a)
    rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return rx @ ry @ rz


def euler_from_matrix(matrix: np.ndarray, *, degrees: bool = False) -> np.ndarray:
    """Convert a rotation matrix to intrinsic XYZ Euler angles (roll, pitch, yaw)."""
    m = np.asarray(matrix, dtype=float)[:3, :3]
    sy = m[0, 2]
    sy = float(np.clip(sy, -1.0, 1.0))
    pitch = np.arcsin(sy)
    if abs(sy) < 1.0 - 1e-6:
        roll = np.arctan2(-m[1, 2], m[2, 2])
        yaw = np.arctan2(-m[0, 1], m[0, 0])
    else:
        # Gimbal lock.
        roll = np.arctan2(m[2, 1], m[1, 1])
        yaw = 0.0
    angles = np.array([roll, pitch, yaw])
    return np.rad2deg(angles) if degrees else angles


# ---------------------------------------------------------------------------
# Pose dataclass
# ---------------------------------------------------------------------------


@dataclass
class TCPPose:
    """Tool-center-point pose: position in metres, orientation as a rotation vector.

    Attributes:
        x, y, z: Cartesian position of the end-effector, in metres.
        rx, ry, rz: Orientation as an axis-angle rotation vector, in radians.
    """

    x: float
    y: float
    z: float
    rx: float = 0.0
    ry: float = 0.0
    rz: float = 0.0

    # -- constructors -------------------------------------------------------
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

    @classmethod
    def from_list(cls, values) -> "TCPPose":
        values = list(values)
        if len(values) != 6:
            raise ValueError("TCPPose.from_list expects [x, y, z, rx, ry, rz]")
        return cls(*map(float, values))

    @classmethod
    def from_dict(cls, data: dict) -> "TCPPose":
        return cls(
            float(data["x"]),
            float(data["y"]),
            float(data["z"]),
            float(data.get("rx", 0.0)),
            float(data.get("ry", 0.0)),
            float(data.get("rz", 0.0)),
        )

    # -- accessors ----------------------------------------------------------
    @property
    def position(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z], dtype=float)

    @property
    def rotvec(self) -> np.ndarray:
        return np.array([self.rx, self.ry, self.rz], dtype=float)

    def to_matrix(self) -> np.ndarray:
        return _compose(self.position, matrix_from_rotvec(self.rotvec))

    def as_quat(self) -> np.ndarray:
        """Orientation as a quaternion ``[x, y, z, w]``."""
        return quat_from_matrix(matrix_from_rotvec(self.rotvec))

    def as_euler(self, *, degrees: bool = False) -> np.ndarray:
        """Orientation as intrinsic XYZ Euler angles (roll, pitch, yaw)."""
        return euler_from_matrix(matrix_from_rotvec(self.rotvec), degrees=degrees)

    def to_list(self) -> list[float]:
        return [self.x, self.y, self.z, self.rx, self.ry, self.rz]

    def to_dict(self) -> dict:
        return {
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "rx": self.rx,
            "ry": self.ry,
            "rz": self.rz,
        }

    def __repr__(self) -> str:  # pragma: no cover - cosmetic
        return (
            f"TCPPose(x={self.x:.4f}, y={self.y:.4f}, z={self.z:.4f}, "
            f"rx={self.rx:.4f}, ry={self.ry:.4f}, rz={self.rz:.4f})"
        )


def _compose(position, rotation_matrix: np.ndarray) -> np.ndarray:
    t = np.eye(4, dtype=float)
    t[:3, :3] = np.asarray(rotation_matrix, dtype=float)
    t[:3, 3] = np.asarray(position, dtype=float).reshape(3)
    return t
