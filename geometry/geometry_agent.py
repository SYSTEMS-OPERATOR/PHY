from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Tuple
import math
import numpy as np

from skeleton.base import BoneSpec


def cylinder_inertia(mass: float, radius: float, length: float) -> List[List[float]]:
    i_xx = 1/12 * mass * (3*radius**2 + length**2)
    i_zz = 0.5 * mass * radius**2
    return [[i_xx, 0.0, 0.0], [0.0, i_xx, 0.0], [0.0, 0.0, i_zz]]


def box_inertia(mass: float, size: Tuple[float, float, float]) -> List[List[float]]:
    x, y, z = size
    i_xx = 1/12 * mass * (y**2 + z**2)
    i_yy = 1/12 * mass * (x**2 + z**2)
    i_zz = 1/12 * mass * (x**2 + y**2)
    return [[i_xx, 0.0, 0.0], [0.0, i_yy, 0.0], [0.0, 0.0, i_zz]]


@dataclass
class GeometryAgent:
    bone: BoneSpec
    dataset: Dict[str, dict]

    def compute(self) -> None:
        dims = self.bone.dimensions
        l = dims.get("length_cm", 1.0)
        w = dims.get("width_cm", l)
        t = dims.get("thickness_cm", w)
        # volume in cm^3
        if self.bone.bone_type == "long":
            radius = w / 2.0
            volume = math.pi * radius**2 * l
            com = (0.0, 0.0, l / 2.0)
            mass = self.bone.material.get("density", 1800) * volume / 1e6
            inertia = cylinder_inertia(mass, radius / 100.0, l / 100.0)
            verts = [(-radius, -radius, 0.0), (radius, radius, l)]
            faces = []
        elif self.bone.bone_type == "flat":
            volume = l * w * 0.2
            com = (0.0, 0.0, t / 2.0)
            mass = self.bone.material.get("density", 1800) * volume / 1e6
            inertia = box_inertia(mass, (l/100.0, w/100.0, t/100.0))
            verts = [(-l/2, -w/2, 0), (l/2, w/2, t)]
            faces = []
        else:
            volume = l * w * t
            com = (0.0, 0.0, t / 2.0)
            mass = self.bone.material.get("density", 1800) * volume / 1e6
            inertia = box_inertia(mass, (l/100.0, w/100.0, t/100.0))
            verts = [(-l/2, -w/2, -t/2), (l/2, w/2, t/2)]
            faces = []

        self.bone.geometry = {
            "verts": verts,
            "faces": faces,
            "V_cm3": volume,
            "COM": com,
            "inertia_kgm2": inertia,
        }

    def recompute(self) -> None:
        self.compute()

