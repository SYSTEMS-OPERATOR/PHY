from __future__ import annotations

from dataclasses import dataclass
from typing import Dict


@dataclass
class WolffAdaptationEngine:
    bone: object
    cycles: int = 0
    baseline_density: float = 0.0

    def record(self, axial_stress: float) -> None:
        self.cycles += 1
        if self.cycles % 100 == 0:
            density = self.bone.material.get("density", self.baseline_density)
            if axial_stress > 1.1:
                density *= 1.01
            else:
                density *= 0.9975
            self.bone.material["density"] = density
