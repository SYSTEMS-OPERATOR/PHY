from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple


@dataclass
class JointSpec:
    """Kinematic link between two bones."""

    name: str
    parent_uid: str
    child_uid: str
    joint_type: str  # 'hinge', 'ball', 'pivot', 'fixed'
    axis: Tuple[float, float, float]
    limit_deg: Tuple[float, float] | Tuple[float, float, float]
    origin_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    origin_rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    @classmethod
    def hinge(
        cls,
        name: str,
        parent_uid: str,
        child_uid: str,
        axis: Tuple[float, float, float] = (0.0, 0.0, 1.0),
        limit: Tuple[float, float] = (-180.0, 180.0),
        origin_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        origin_rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> "JointSpec":
        return cls(name, parent_uid, child_uid, "hinge", axis, limit, origin_xyz, origin_rpy)

    @classmethod
    def ball(
        cls,
        name: str,
        parent_uid: str,
        child_uid: str,
        limit: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        origin_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        origin_rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> "JointSpec":
        return cls(name, parent_uid, child_uid, "ball", (0.0, 0.0, 0.0), limit, origin_xyz, origin_rpy)

    @classmethod
    def fixed(
        cls,
        name: str,
        parent_uid: str,
        child_uid: str,
        origin_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        origin_rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> "JointSpec":
        return cls(name, parent_uid, child_uid, "fixed", (0.0, 0.0, 0.0), (0.0, 0.0), origin_xyz, origin_rpy)
