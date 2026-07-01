"""Persistent, named pose library backed by a JSON file.

A saved pose records both the joint configuration (the source of truth for
replaying a motion exactly) and the Cartesian TCP pose (for inspection and
Cartesian replay via inverse kinematics).
"""

from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

from .exceptions import PoseNotFoundError
from .poses import TCPPose


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

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict) -> "SavedPose":
        return cls(
            name=data["name"],
            joints={k: float(v) for k, v in data["joints"].items()},
            tcp=data.get("tcp"),
            robot_id=data.get("robot_id"),
            created_at=data.get("created_at", time.time()),
        )


class PoseStore:
    """Reads/writes a dictionary of :class:`SavedPose` to a JSON file on disk."""

    def __init__(self, path: str | Path):
        self.path = Path(path).expanduser()
        self._poses: dict[str, SavedPose] = {}
        self._load()

    # -- persistence --------------------------------------------------------
    def _load(self) -> None:
        if not self.path.is_file():
            self._poses = {}
            return
        with open(self.path) as f:
            raw = json.load(f)
        self._poses = {name: SavedPose.from_dict(entry) for name, entry in raw.items()}

    def _flush(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        serialisable = {name: pose.to_dict() for name, pose in self._poses.items()}
        tmp = self.path.with_suffix(self.path.suffix + ".tmp")
        with open(tmp, "w") as f:
            json.dump(serialisable, f, indent=2)
        tmp.replace(self.path)

    # -- mutation -----------------------------------------------------------
    def save(self, pose: SavedPose) -> SavedPose:
        self._poses[pose.name] = pose
        self._flush()
        return pose

    def delete(self, name: str) -> None:
        if name not in self._poses:
            raise PoseNotFoundError(name)
        del self._poses[name]
        self._flush()

    # -- access -------------------------------------------------------------
    def get(self, name: str) -> SavedPose:
        if name not in self._poses:
            raise PoseNotFoundError(name)
        return self._poses[name]

    def names(self) -> list[str]:
        return sorted(self._poses.keys())

    def all(self) -> dict[str, SavedPose]:
        return dict(self._poses)

    def __contains__(self, name: object) -> bool:
        return name in self._poses

    def __len__(self) -> int:
        return len(self._poses)
