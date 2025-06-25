from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Tuple

from skeleton.base import BoneSpec


@dataclass
class GeometryData:
    verts: List[Tuple[float, float, float]]
    faces: List[Tuple[int, int, int]]
    volume_cm3: float
    com: Tuple[float, float, float]
    inertia_kgm2: Tuple[float, float, float]


class GeometryAgent:
    """Generate simple geometry and inertial properties for bones."""

    def __init__(self, dataset: Dict[str, dict] | None = None) -> None:
        self.dataset = dataset or {}

    def build_geometry(self, bone: BoneSpec) -> GeometryData:
        dims = bone.dimensions
        length = dims.get("length_cm") or 1.0
        width = dims.get("width_cm") or 1.0
        thickness = dims.get("thickness_cm") or width

        if bone.bone_type == "long":
            radius = width * 0.5
            volume = math.pi * radius ** 2 * length
            com = (0.0, 0.0, length * 0.5)
            i = (0.5 * bone.material.get("density", 1000) * volume / 1000 * radius ** 2,
                 0.5 * bone.material.get("density", 1000) * volume / 1000 * radius ** 2,
                 (1/12) * bone.material.get("density", 1000) * volume / 1000 * length ** 2)
            verts = [(0, 0, 0), (0, 0, length)]
            faces = []
        else:
            # simple box
            volume = length * width * thickness
            com = (length/2, width/2, thickness/2)
            mass = bone.material.get("density", 1000) * volume / 1000
            i = (
                (1/12) * mass * (width**2 + thickness**2),
                (1/12) * mass * (length**2 + thickness**2),
                (1/12) * mass * (length**2 + width**2),
            )
            verts = [(0, 0, 0), (length, 0, 0), (length, width, 0), (0, width, 0),
                     (0, 0, thickness), (length, 0, thickness), (length, width, thickness), (0, width, thickness)]
            faces = [(0,1,2), (0,2,3), (4,5,6), (4,6,7)]

        geom = GeometryData(verts, faces, volume, com, i)
        bone.geometry = {
            "verts": verts,
            "faces": faces,
            "V_cm3": volume,
            "COM": com,
            "inertia_kgm2": i,
        }
        return geom

    def recompute(self, bone: BoneSpec) -> GeometryData:
        return self.build_geometry(bone)
