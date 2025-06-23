from __future__ import annotations

from typing import Dict, List, Optional

from .base import BoneSpec


class SkeletonField:
    """Container tracking an entangled network of bones."""

    def __init__(self, bones: Optional[List[BoneSpec]] = None) -> None:
        self.bones: Dict[str, BoneSpec] = {}
        if bones:
            for bone in bones:
                self.register(bone)
            self.build_entanglement()

    def register(self, bone: BoneSpec) -> None:
        self.bones[bone.domain_id] = bone

    def get(self, domain_id: str) -> Optional[BoneSpec]:
        return self.bones.get(domain_id)

    def build_entanglement(self) -> None:
        name_map: Dict[str, List[BoneSpec]] = {}
        for bone in self.bones.values():
            name_map.setdefault(bone.name.lower(), []).append(bone)
        for bone in self.bones.values():
            for art in bone.articulations:
                partner_name = art.get("bone", "").lower()
                for partner in name_map.get(partner_name, []):
                    bone.entangle(partner)

    def broadcast(self, voltage: float, signal_type: str = "EMG") -> None:
        for bone in self.bones.values():
            bone.emit_signal(None, voltage, signal_type)

    def total_voltage(self) -> float:
        return sum(b.voltage_potential for b in self.bones.values())

    def summary(self) -> Dict[str, float]:
        return {d: b.voltage_potential for d, b in self.bones.items()}
