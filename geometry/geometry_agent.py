from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple

from skeleton.base import BoneSpec


@dataclass
class GeometryAgent:
    """Generate simple geometry and inertia for a :class:`BoneSpec`."""

    bone: BoneSpec

    def compute(self) -> None:
        """Attach vertices, faces, volume, COM and inertia to the bone."""
        dims = self.bone.dimensions
        length = dims.get("length_cm")
        width = dims.get("width_cm")
        if length is None or width is None:
            return
        radius = width / 2.0
        verts, faces = self._cylinder_mesh(radius, length)
        volume = math.pi * radius ** 2 * length
        com = (0.0, 0.0, length / 2.0)
        mass = (self.bone.material.get("density", 1800.0) * volume) / 1e6
        r_m = radius / 100.0
        l_m = length / 100.0
        ixx = (1.0 / 12.0) * mass * (3 * r_m ** 2 + l_m ** 2)
        iyy = ixx
        izz = 0.5 * mass * r_m ** 2
        inertia = ((ixx, 0.0, 0.0), (0.0, iyy, 0.0), (0.0, 0.0, izz))
        self.bone.geometry = {
            "verts": verts,
            "faces": faces,
            "V_cm3": volume,
            "COM": com,
            "inertia_kgm2": inertia,
        }

    def _cylinder_mesh(
        self, radius_cm: float, length_cm: float, segments: int = 8
    ) -> Tuple[List[Tuple[float, float, float]], List[Tuple[int, int, int, int]]]:
        verts: List[Tuple[float, float, float]] = []
        faces: List[Tuple[int, int, int, int]] = []
        for i in range(segments):
            angle = 2 * math.pi * i / segments
            verts.append((radius_cm * math.cos(angle), radius_cm * math.sin(angle), 0.0))
        offset = len(verts)
        for i in range(segments):
            angle = 2 * math.pi * i / segments
            verts.append((radius_cm * math.cos(angle), radius_cm * math.sin(angle), length_cm))
        for i in range(segments):
            j = (i + 1) % segments
            faces.append((i, j, offset + j, offset + i))
        return verts, faces
