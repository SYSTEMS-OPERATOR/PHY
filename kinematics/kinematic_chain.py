from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional
import numpy as np

from skeleton.base import BoneSpec
from joints.joint_spec import JointSpec


def rpy_matrix(rpy: Tuple[float, float, float]) -> np.ndarray:
    r, p, y = rpy
    Rx = np.array([
        [1, 0, 0, 0],
        [0, np.cos(r), -np.sin(r), 0],
        [0, np.sin(r), np.cos(r), 0],
        [0, 0, 0, 1],
    ])
    Ry = np.array([
        [np.cos(p), 0, np.sin(p), 0],
        [0, 1, 0, 0],
        [-np.sin(p), 0, np.cos(p), 0],
        [0, 0, 0, 1],
    ])
    Rz = np.array([
        [np.cos(y), -np.sin(y), 0, 0],
        [np.sin(y), np.cos(y), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ])
    return Rz @ Ry @ Rx


def trans_matrix(xyz: Tuple[float, float, float]) -> np.ndarray:
    T = np.eye(4)
    T[:3, 3] = xyz
    return T


def axis_angle_matrix(axis: Tuple[float, float, float], angle: float) -> np.ndarray:
    ax = np.array(axis, dtype=float)
    ax = ax / (np.linalg.norm(ax) + 1e-8)
    c = np.cos(angle)
    s = np.sin(angle)
    x, y, z = ax
    R = np.array([
        [c + x*x*(1-c), x*y*(1-c) - z*s, x*z*(1-c) + y*s, 0],
        [y*x*(1-c) + z*s, c + y*y*(1-c), y*z*(1-c) - x*s, 0],
        [z*x*(1-c) - y*s, z*y*(1-c) + x*s, c + z*z*(1-c), 0],
        [0, 0, 0, 1],
    ])
    return R


@dataclass
class KinematicChain:
    bones: Dict[str, BoneSpec]
    joints: List[JointSpec]
    root: str

    def forward_kinematics(self, angles: Dict[str, float]) -> Dict[str, np.ndarray]:
        transforms: Dict[str, np.ndarray] = {self.root: np.eye(4)}
        joint_map = {j.child_uid: j for j in self.joints}
        pending = list(joint_map.keys())
        while pending:
            child = pending.pop(0)
            joint = joint_map[child]
            parent_tf = transforms.get(joint.parent_uid)
            if parent_tf is None:
                pending.append(child)
                continue
            T = parent_tf @ trans_matrix(joint.origin_xyz) @ rpy_matrix(joint.origin_rpy)
            if joint.joint_type != 'fixed':
                angle = angles.get(joint.name, 0.0)
                T = T @ axis_angle_matrix(joint.axis, angle)
            length = self.bones[child].dimensions.get('length_cm', 0.0)
            T = T @ trans_matrix((length, 0, 0))
            transforms[child] = T
        return transforms

    def inverse_kinematics(self, target: np.ndarray, end_effector: str, initial: Optional[Dict[str, float]] = None, iters: int = 50) -> Dict[str, float]:
        if initial is None:
            angles = {j.name: 0.0 for j in self.joints}
        else:
            angles = dict(initial)
        if len(self.joints) == 2:
            l1 = self.bones[self.joints[0].child_uid].dimensions.get('length_cm', 0.0)
            l2 = self.bones[self.joints[1].child_uid].dimensions.get('length_cm', 0.0)
            x, z = target[0], target[2]
            r2 = x**2 + z**2
            cos2 = (r2 - l1**2 - l2**2) / (2*l1*l2)
            cos2 = max(min(cos2, 1.0), -1.0)
            theta2 = np.arccos(cos2)
            k1 = l1 + l2*np.cos(theta2)
            k2 = l2*np.sin(theta2)
            theta1 = np.arctan2(z, x) - np.arctan2(k2, k1)
            angles[self.joints[0].name] = theta1
            angles[self.joints[1].name] = theta2
            return angles
        for _ in range(iters):
            tfs = self.forward_kinematics(angles)
            pos = tfs[end_effector][:3, 3]
            err = target - pos
            if np.linalg.norm(err) < 0.02:
                break
            for j in reversed(self.joints):
                if j.joint_type == 'fixed':
                    continue
                parent_tf = tfs[j.parent_uid]
                joint_pos = (parent_tf @ trans_matrix(j.origin_xyz))[:3, 3]
                axis_world = parent_tf[:3, :3] @ np.array(j.axis)
                jac = np.cross(axis_world, target - joint_pos)
                step = 0.1 * jac.dot(err)
                angles[j.name] += step
        return angles

    def _joint_index(self, name: str) -> int:
        for i, j in enumerate(self.joints):
            if j.name == name:
                return i
        return -1

