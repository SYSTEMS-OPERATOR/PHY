from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np

from skeleton.base import BoneSpec
from joints.joint_spec import JointSpec


def _rot_from_axis_angle(axis: Tuple[float, float, float], angle_rad: float) -> np.ndarray:
    x, y, z = axis
    norm = math.sqrt(x * x + y * y + z * z)
    if norm == 0:
        return np.eye(3)
    x /= norm
    y /= norm
    z /= norm
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    C = 1 - c
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


def _matrix_from_rpy_xyz(rpy: Tuple[float, float, float], xyz: Tuple[float, float, float]) -> np.ndarray:
    r, p, y = rpy
    sr, cr = math.sin(r), math.cos(r)
    sp, cp = math.sin(p), math.cos(p)
    sy, cy = math.sin(y), math.cos(y)
    rot = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )
    mat = np.eye(4)
    mat[:3, :3] = rot
    mat[:3, 3] = xyz
    return mat


@dataclass
class KinematicChain:
    bones: Dict[str, BoneSpec]
    joints: List[JointSpec]

    def __post_init__(self) -> None:
        self.child_map: Dict[str, List[JointSpec]] = {}
        for j in self.joints:
            self.child_map.setdefault(j.parent_uid, []).append(j)
        self.joint_order = [j.name for j in self.joints]

    def forward_kinematics(self, angles: Dict[str, float]) -> Dict[str, np.ndarray]:
        transforms: Dict[str, np.ndarray] = {}
        root_ids = [uid for uid in self.bones if uid not in {j.child_uid for j in self.joints}]
        for rid in root_ids:
            transforms[rid] = np.eye(4)
            self._fk_recursive(rid, transforms, angles)
        return transforms

    def _fk_recursive(self, uid: str, transforms: Dict[str, np.ndarray], angles: Dict[str, float]) -> None:
        for j in self.child_map.get(uid, []):
            parent_tf = transforms[uid]
            T_origin = _matrix_from_rpy_xyz(j.origin_rpy, j.origin_xyz)
            ang = math.radians(angles.get(j.name, 0.0))
            if j.joint_type != "fixed":
                R = _rot_from_axis_angle(j.axis, ang)
            else:
                R = np.eye(3)
            T_rot = np.eye(4)
            T_rot[:3, :3] = R
            child_tf = parent_tf @ T_origin @ T_rot
            transforms[j.child_uid] = child_tf
            self._fk_recursive(j.child_uid, transforms, angles)

    def inverse_kinematics(
        self,
        end_effector_uid: str,
        target_xyz: Tuple[float, float, float],
        initial: Dict[str, float] | None = None,
        max_iter: int = 50,
    ) -> Dict[str, float]:
        angles = np.array([math.radians(initial.get(n, 0.0)) if initial else 0.0 for n in self.joint_order])
        for _ in range(max_iter):
            tf = self.forward_kinematics({n: math.degrees(a) for n, a in zip(self.joint_order, angles)})
            pos = tf[end_effector_uid][:3, 3]
            err = np.array(target_xyz) - pos
            if np.linalg.norm(err) < 0.02:
                break
            J = np.zeros((3, len(angles)))
            eps = 1e-3
            for i in range(len(angles)):
                a = angles.copy()
                a[i] += eps
                tf_p = self.forward_kinematics({n: math.degrees(v) for n, v in zip(self.joint_order, a)})
                pos_p = tf_p[end_effector_uid][:3, 3]
                J[:, i] = (pos_p - pos) / eps
            damp = 1e-2
            angles += np.linalg.pinv(J.T @ J + damp * np.eye(len(angles))) @ J.T @ err
        return {n: math.degrees(a) for n, a in zip(self.joint_order, angles)}
