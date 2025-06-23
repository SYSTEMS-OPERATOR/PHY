from __future__ import annotations

from typing import Dict, List

from .base import BoneSpec

class SkeletonField:
    """Network field managing bioelectric signal propagation."""

    def __init__(self, bones: List[BoneSpec]):
        self.bones: Dict[str, BoneSpec] = {b.domain_id: b for b in bones}

    def broadcast(self, voltage: float, signal_type: str = "EMG") -> None:
        signal = {"voltage": voltage, "type": signal_type}
        for bone in self.bones.values():
            bone.receive_signal(signal, "FIELD")

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

    def status(self) -> Dict[str, Dict[str, object]]:
        return {d: {"voltage": b.voltage_potential, "links": list(b.entanglement_links)} for d, b in self.bones.items()}

    def health(self) -> Dict[str, bool]:
        """Return health status for each bone."""
        return {d: b.is_healthy() for d, b in self.bones.items()}

    def faults(self) -> Dict[str, List[str]]:
        """Return fault lists per bone."""
        return {d: b.report_faults() for d, b in self.bones.items() if not b.is_healthy()}
