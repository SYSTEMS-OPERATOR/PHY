from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Tuple
import math
import numpy as np

from skeleton.base import BoneSpec
from joints.joint_spec import JointSpec


def rot_from_axis_angle(axis: Tuple[float, float, float], angle_rad: float) -> np.ndarray:
    ax = np.array(axis, dtype=float)
    if np.linalg.norm(ax) == 0:
        return np.eye(3)
    ax = ax / np.linalg.norm(ax)
    x, y, z = ax
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    C = 1 - c
    R = np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])
    return R


def transform_matrix(R: np.ndarray, t: Tuple[float, float, float]) -> np.ndarray:
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = np.array(t)
    return M


@dataclass
class KinematicChain:
    bones: Dict[str, BoneSpec]
    joints: List[JointSpec]
    root_uid: str

    def forward_kinematics(self, angles: Dict[str, float]) -> Dict[str, np.ndarray]:
        T = {self.root_uid: np.eye(4)}
        queue = [self.root_uid]
        while queue:
            parent = queue.pop(0)
            for j in self.joints:
                if j.parent_uid != parent:
                    continue
                ang = angles.get(j.name, 0.0)
                rpy = tuple(math.radians(v) for v in j.origin_rpy)
                R_origin = (
                    rot_from_axis_angle((0, 0, 1), rpy[2])
                    @ rot_from_axis_angle((0, 1, 0), rpy[1])
                    @ rot_from_axis_angle((1, 0, 0), rpy[0])
                )
                origin = transform_matrix(R_origin, j.origin_xyz)
                R = rot_from_axis_angle(j.axis, math.radians(ang))
                M = T[parent] @ origin @ transform_matrix(R, (0, 0, 0))
                T[j.child_uid] = M
                queue.append(j.child_uid)
        return T

    def end_effector_position(self, angles: Dict[str, float], end_uid: str) -> np.ndarray:
        T = self.forward_kinematics(angles)
        base = T[end_uid]
        length = self.bones[end_uid].dimensions.get('length_cm', 0.0) / 100
        tip = base @ np.array([0, 0, length, 1.0])
        return tip[:3]

    def inverse_kinematics(self, end_uid: str, target: Tuple[float, float, float],
                           initial: Dict[str, float] | None = None,
                           max_iter: int = 50, tol: float = 0.02) -> Dict[str, float]:
        if initial is None:
            initial = {j.name: 0.0 for j in self.joints}
        angles = dict(initial)
        joint_names = [j.name for j in self.joints]
        dq = np.zeros(len(joint_names))
        for _ in range(max_iter):
            pos = self.end_effector_position(angles, end_uid)
            err = np.array(target) - pos
            if np.linalg.norm(err) < tol:
                break
            J = []
            for name in joint_names:
                ang_eps = dict(angles)
                ang_eps[name] += 0.001
                pos_eps = self.end_effector_position(ang_eps, end_uid)
                J.append((pos_eps - pos) / 0.001)
            J = np.array(J).T
            dq = np.linalg.pinv(J) @ err
        for i, name in enumerate(joint_names):
            angles[name] += math.degrees(dq[i])
        return angles

    def center_of_mass(self, angles: Dict[str, float]) -> np.ndarray:
        """Return the world-space center of mass for the chain."""
        transforms = self.forward_kinematics(angles)
        com_sum = np.zeros(3)
        mass_sum = 0.0
        for uid, T in transforms.items():
            bone = self.bones[uid]
            com_local = bone.geometry.get("COM", (0.0, 0.0, 0.0))
            com_world = T @ np.array([com_local[0], com_local[1], com_local[2], 1.0])
            mass = bone.mass_kg() or 0.0
            com_sum += mass * com_world[:3]
            mass_sum += mass
        if mass_sum == 0.0:
            return np.zeros(3)
        return com_sum / mass_sum

