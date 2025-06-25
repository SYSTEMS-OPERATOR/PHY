from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict


@dataclass
class WolffAdaptationEngine:
    """Track stress cycles and modify bone density."""

    bones: Dict[str, "BoneSpec"]
    cycles: Dict[str, int] = field(default_factory=dict)
    mean_stress: Dict[str, float] = field(default_factory=dict)

    def record(self, loads: Dict[str, float]) -> None:
        for uid, stress in loads.items():
            self.cycles[uid] = self.cycles.get(uid, 0) + 1
            self.mean_stress[uid] = 0.9 * self.mean_stress.get(uid, stress) + 0.1 * stress
            if self.cycles[uid] % 1000 == 0:
                bone = self.bones[uid]
                baseline = bone.material.get("density", 1800.0)
                current = bone.material.get("density", baseline)
                if self.mean_stress[uid] > baseline * 1.1:
                    new_d = min(current * 1.01, baseline * 1.2)
                else:
                    new_d = max(current * 0.9975, baseline * 0.8)
                bone.material["density"] = new_d
