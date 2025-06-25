from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple


@dataclass
class JointSpec:
    name: str
    parent_uid: str
    child_uid: str
    joint_type: str
    axis: Tuple[float, float, float] = (0.0, 0.0, 1.0)
    limit_deg: Tuple[float, float] | Tuple[float, float, float] | None = None
    origin_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    origin_rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)


def hinge(name: str, parent: str, child: str, axis=(0, 0, 1), limit=(-180, 180), origin=(0,0,0), rpy=(0,0,0)) -> JointSpec:
    return JointSpec(name, parent, child, 'hinge', axis, limit, origin, rpy)


def ball(name: str, parent: str, child: str, limit=(0,0,0), origin=(0,0,0), rpy=(0,0,0)) -> JointSpec:
    return JointSpec(name, parent, child, 'ball', (0,0,0), limit, origin, rpy)


def fixed(name: str, parent: str, child: str, origin=(0,0,0), rpy=(0,0,0)) -> JointSpec:
    return JointSpec(name, parent, child, 'fixed', (0,0,0), None, origin, rpy)

