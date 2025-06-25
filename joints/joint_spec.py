from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple


@dataclass
class JointSpec:
    name: str
    parent_uid: str
    child_uid: str
    joint_type: str  # 'hinge', 'ball', 'pivot', 'fixed'
    axis: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    limit_deg: Tuple[float, ...] = (0.0, 0.0)
    origin_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    origin_rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)


def hinge(name: str, parent_uid: str, child_uid: str, axis=(1.0, 0.0, 0.0),
          limit=(-135.0, 0.0), origin_xyz=(0.0, 0.0, 0.0),
          origin_rpy=(0.0, 0.0, 0.0)) -> JointSpec:
    return JointSpec(name, parent_uid, child_uid, 'hinge', axis, limit,
                     origin_xyz, origin_rpy)


def pivot(name: str, parent_uid: str, child_uid: str, axis=(0.0, 0.0, 1.0),
          limit=(-90.0, 90.0), origin_xyz=(0.0, 0.0, 0.0),
          origin_rpy=(0.0, 0.0, 0.0)) -> JointSpec:
    return JointSpec(name, parent_uid, child_uid, 'pivot', axis, limit,
                     origin_xyz, origin_rpy)


def ball(name: str, parent_uid: str, child_uid: str,
         limit=(0.0, 0.0, 0.0), origin_xyz=(0.0, 0.0, 0.0),
         origin_rpy=(0.0, 0.0, 0.0)) -> JointSpec:
    return JointSpec(name, parent_uid, child_uid, 'ball', (0.0, 0.0, 0.0), limit,
                     origin_xyz, origin_rpy)


def fixed(name: str, parent_uid: str, child_uid: str,
          origin_xyz=(0.0, 0.0, 0.0), origin_rpy=(0.0, 0.0, 0.0)) -> JointSpec:
    return JointSpec(name, parent_uid, child_uid, 'fixed', (0.0, 0.0, 0.0),
                     (0.0, 0.0), origin_xyz, origin_rpy)

