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


    def summary(self) -> Dict[str, float]:
        return {d: b.voltage_potential for d, b in self.bones.items()}


    def propagate(self, from_domain: str, signal: Dict[str, float]) -> None:
        origin = self.bones.get(from_domain)
        if not origin:
            return
        visited = set()

        def _prop(b: BoneSpec):
            if b.domain_id in visited:
                return
            visited.add(b.domain_id)
            for link_id in b.entanglement_links:
                other = self.bones.get(link_id)
                if other:
                    other.receive_signal(signal, b.domain_id)
                    _prop(other)

        _prop(origin)

    def total_voltage(self) -> float:
        """Return the summed voltage potential of all bones."""
        return sum(b.voltage_potential for b in self.bones.values())

    def meta_breath(self, word: str) -> None:
        """Invoke a metaphysical intent across all bones."""
        for bone in self.bones.values():
            bone.transcend(intent=word)

    def collapse(self) -> None:
        """Remove all entanglement links between bones."""
        for bone in self.bones.values():
            bone.entanglement_links.clear()

    def status(self) -> Dict[str, Dict[str, object]]:
        return {d: {"voltage": b.voltage_potential, "links": list(b.entanglement_links)} for d, b in self.bones.items()}

    def health(self) -> Dict[str, bool]:
        """Return health status for each bone."""
        return {d: b.is_healthy() for d, b in self.bones.items()}

    def faults(self) -> Dict[str, List[str]]:
        """Return fault lists per bone."""
        return {d: b.report_faults() for d, b in self.bones.items() if not b.is_healthy()}

