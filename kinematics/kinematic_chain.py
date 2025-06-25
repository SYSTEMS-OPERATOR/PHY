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
        joint_map = {j.child_uid: j for j in self.joints}
        queue = [self.root_uid]
        while queue:
            parent = queue.pop(0)
            for j in self.joints:
                if j.parent_uid != parent:
                    continue
                ang = angles.get(j.name, 0.0)
                R_origin = rot_from_axis_angle((0, 0, 1), 0)
                R = rot_from_axis_angle(j.axis, math.radians(ang))
                origin = transform_matrix(np.eye(3), j.origin_xyz)
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

