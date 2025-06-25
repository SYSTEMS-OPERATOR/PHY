from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Iterable, List

from .receptor_spec import ReceptorSpec
from soft.muscle_agent import MuscleAgent
from soft.ligament_agent import LigamentAgent
from skeleton.base import BoneSpec


@dataclass
class SensorAgent:
    """Collect receptor firing rates based on tissue state."""

    receptors: List[ReceptorSpec]
    muscles: Iterable[MuscleAgent] = field(default_factory=list)
    ligaments: Iterable[LigamentAgent] = field(default_factory=list)
    bones: Iterable[BoneSpec] = field(default_factory=list)

    def __post_init__(self) -> None:
        self.muscles = {m.spec.name: m for m in self.muscles}
        self.ligaments = {l.joint_name: l for l in self.ligaments}
        self.bones = {b.unique_id: b for b in self.bones}
        self.firing: Dict[str, float] = {r.name: 0.0 for r in self.receptors}
        self._baseline_len: Dict[str, float] = {}

    def update(self, dt: float) -> Dict[str, float]:
        for r in self.receptors:
            if r.type == "muscle_spindle":
                m = self.muscles.get(r.location.get("muscle"))
                if m is None:
                    continue
                base = self._baseline_len.setdefault(r.name, m.length_m)
                strain = m.length_m - base
                self.firing[r.name] = max((strain - r.threshold) * r.signal_gain, 0.0)
            elif r.type == "GTO":
                m = self.muscles.get(r.location.get("muscle"))
                if m is None:
                    continue
                tension = m.spec.activation
                self.firing[r.name] = max((tension - r.threshold) * r.signal_gain, 0.0)
            elif r.type == "nociceptor":
                uid = r.location.get("bone_uid")
                bone = self.bones.get(uid)
                damage = getattr(bone, "damage", 0.0) if bone else 0.0
                self.firing[r.name] = damage * r.signal_gain
            else:
                self.firing[r.name] = 0.0
        return dict(self.firing)
