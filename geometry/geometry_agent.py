from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple
import math

from skeleton.base import BoneSpec


def cylinder_inertia(mass: float, radius: float, length: float) -> List[List[float]]:
    i_xx = 0.5 * mass * radius ** 2
    i_yy = (1 / 12) * mass * (3 * radius ** 2 + length ** 2)
    i_zz = i_yy
    return [[i_xx, 0.0, 0.0], [0.0, i_yy, 0.0], [0.0, 0.0, i_zz]]


def box_inertia(mass: float, x: float, y: float, z: float) -> List[List[float]]:
    i_xx = (1 / 12) * mass * (y ** 2 + z ** 2)
    i_yy = (1 / 12) * mass * (x ** 2 + z ** 2)
    i_zz = (1 / 12) * mass * (x ** 2 + y ** 2)
    return [[i_xx, 0.0, 0.0], [0.0, i_yy, 0.0], [0.0, 0.0, i_zz]]


@dataclass
class GeometryAgent:
    bone: BoneSpec

    def compute(self) -> None:
        dims = self.bone.dimensions
        l = dims.get("length_cm")
        w = dims.get("width_cm")
        t = dims.get("thickness_cm")
        if l is None or w is None:
            return
        shape = self.bone.bone_type
        density = self.bone.material.get("density", 1800.0)
        if shape == "long":
            radius_m = (w * 0.5) / 100
            length_m = l / 100
            volume_m3 = math.pi * radius_m ** 2 * length_m
            mass = density * volume_m3
            inertia = cylinder_inertia(mass, radius_m, length_m)
            com = (0.0, 0.0, length_m / 2)
            self.bone.geometry.update({
                "type": "cylinder",
                "radius_m": radius_m,
                "length_m": length_m,
                "verts": [],
                "faces": [],
                "V_cm3": volume_m3 * 1e6,
                "COM": com,
                "inertia_kgm2": inertia,
            })
        else:
            if t is None:
                return
            x = w / 100
            y = t / 100
            z = l / 100
            volume_m3 = x * y * z
            mass = density * volume_m3
            inertia = box_inertia(mass, x, y, z)
            com = (x / 2, y / 2, z / 2)
            self.bone.geometry.update({
                "type": "box",
                "width_m": x,
                "thickness_m": y,
                "length_m": z,
                "verts": [],
                "faces": [],
                "V_cm3": volume_m3 * 1e6,
                "COM": com,
                "inertia_kgm2": inertia,
            })

