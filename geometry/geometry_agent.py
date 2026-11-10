from __future__ import annotations

from dataclasses import dataclass
from typing import List
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
    """Attach primitive geometry information to a :class:`BoneSpec`."""

    bone: BoneSpec

    def compute(self) -> None:
        """Compute and store the bone geometry and inertia."""
        # Use BoneSpec's canonical conversion and alias checks. Named body or
        # canal spans never substitute for missing whole-bone dimensions.
        dims = self.bone._dimensions_mm()
        l = dims.get("length_mm")
        w = dims.get("width_mm")
        t = dims.get("thickness_mm")
        if l is None or w is None:
            return
        sizes = (l, w) if self.bone.bone_type == "long" else (l, w, t)
        if any(value is None for value in sizes):
            return
        if any(not math.isfinite(value) or value <= 0 for value in sizes):
            raise ValueError("Primitive dimensions must be finite and positive")
        shape = self.bone.bone_type
        density = self.bone.material.get("density", 1800.0)

        if shape == "long":
            radius_m = (w * 0.5) / 1000
            length_m = l / 1000
            volume_m3 = math.pi * radius_m ** 2 * length_m
            mass = density * volume_m3
            inertia = cylinder_inertia(mass, radius_m, length_m)
            com = (0.0, 0.0, length_m / 2)
            geom = {
                "type": "cylinder",
                "radius_m": radius_m,
                "length_m": length_m,
            }
        elif shape == "flat" and t is not None:
            x = w / 1000
            y = t / 1000
            z = l / 1000
            volume_m3 = x * y * z
            mass = density * volume_m3
            inertia = box_inertia(mass, x, y, z)
            com = (x / 2, y / 2, z / 2)
            geom = {
                "type": "plate",
                "width_m": x,
                "thickness_m": y,
                "length_m": z,
            }
        else:
            # irregular or generic box approximation
            if t is None:
                return
            x = w / 1000
            y = t / 1000
            z = l / 1000
            volume_m3 = x * y * z
            mass = density * volume_m3
            inertia = box_inertia(mass, x, y, z)
            com = (x / 2, y / 2, z / 2)
            geom = {
                "type": "box",
                "width_m": x,
                "thickness_m": y,
                "length_m": z,
                "irregular": shape == "irregular",
            }

        geom.update({
            "verts": [],
            "faces": [],
            "V_cm3": volume_m3 * 1e6,
            "COM": com,
            "inertia_kgm2": inertia,
        })
        self.bone.geometry.update(geom)

    def recompute(self) -> None:
        """Clear stored geometry and recompute with current attributes."""
        self.bone.geometry.clear()
        self.compute()

