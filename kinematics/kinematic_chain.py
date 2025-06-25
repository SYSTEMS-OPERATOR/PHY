from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Tuple
import math

from skeleton.base import BoneSpec
from joints.joint_spec import JointSpec


def _rot_z(theta: float) -> List[List[float]]:
    c = math.cos(theta)
    s = math.sin(theta)
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


def _mat_mult(a: List[List[float]], b: List[List[float]]) -> List[List[float]]:
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _apply(T: List[List[float]], v: Tuple[float, float, float]) -> Tuple[float, float, float]:
    return (
        T[0][0]*v[0] + T[0][1]*v[1] + T[0][2]*v[2],
        T[1][0]*v[0] + T[1][1]*v[1] + T[1][2]*v[2],
        T[2][0]*v[0] + T[2][1]*v[1] + T[2][2]*v[2],
    )


@dataclass
class Link:
    bone: BoneSpec
    joint: JointSpec | None
    children: List['Link'] | None = None

    def __post_init__(self) -> None:
        if self.children is None:
            self.children = []


class KinematicChain:
    def __init__(self, root: BoneSpec, joints: List[JointSpec], bones: Dict[str, BoneSpec]) -> None:
        self.root = root
        self.joints = {j.child_uid: j for j in joints}
        self.bones = bones
        self.links = self._build_links(root)

    def _build_links(self, bone: BoneSpec) -> Link:
        joint = self.joints.get(bone.unique_id)
        link = Link(bone, joint)
        for j in self.joints.values():
            if j.parent_uid == bone.unique_id:
                link.children.append(self._build_links(self.bones[j.child_uid]))
        return link

    def forward_kinematics(self, angles: Dict[str, float]) -> Dict[str, Tuple[List[List[float]], Tuple[float, float, float]]]:
        out: Dict[str, Tuple[List[List[float]], Tuple[float, float, float]]] = {}

        def _solve(link: Link, R: List[List[float]], p: Tuple[float, float, float]):
            j = link.joint
            cur_R = R
            cur_p = p
            if j:
                theta = math.radians(angles.get(j.name, 0.0))
                rot = _rot_z(theta)
                cur_R = _mat_mult(R, rot)
                cur_p = _apply(R, j.origin_xyz)
                cur_p = (p[0]+cur_p[0], p[1]+cur_p[1], p[2]+cur_p[2])
            out[link.bone.unique_id] = (cur_R, cur_p)
            for c in link.children:
                _solve(c, cur_R, cur_p)

        _solve(self.links, [[1,0,0],[0,1,0],[0,0,1]], (0.0,0.0,0.0))
        return out

    def inverse_kinematics(self, target: Tuple[float, float, float]) -> Dict[str, float]:
        raise RuntimeError('ikpy not available')
