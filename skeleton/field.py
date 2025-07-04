from __future__ import annotations

from typing import Dict, List, Optional, Tuple

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
        """Broadcast a signal to all bones simultaneously."""
        regulated_values: Dict[str, float] = {}
        for bone in self.bones.values():
            regulated = bone.marrow.regulate(voltage, signal_type)
            bone.voltage_potential = regulated
            regulated_values[bone.domain_id] = regulated

        for bone in self.bones.values():
            value = regulated_values[bone.domain_id]
            for link_id in bone.entanglement_links:
                other = self.bones.get(link_id)
                if other is not None:
                    other.receive_signal(value, signal_type, from_domain=bone.domain_id)


    def summary(self) -> Dict[str, float]:
        return {d: b.voltage_potential for d, b in self.bones.items()}


    def propagate(self, from_domain: str, signal: Dict[str, float]) -> None:
        """Propagate a signal through the entanglement network."""
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
                if other is None:
                    continue
                other.receive_signal_packet(signal, b.domain_id)
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

    def audit_metrics(self) -> Dict[str, Dict[str, Tuple[Optional[float], Optional[float]]]]:
        """Return dataset metric discrepancies for all bones."""
        report: Dict[str, Dict[str, Tuple[Optional[float], Optional[float]]]] = {}
        for bone in self.bones.values():
            diff = bone.validate_metrics()
            if diff:
                report[bone.unique_id] = diff
        return report


    # Visualization helpers
    def to_render_nodes(self, color_by: str = 'material') -> List['SkeletonRenderNode']:
        from .visualization import SkeletonRenderNode

        nodes: List[SkeletonRenderNode] = []
        for bone in self.bones.values():
            nodes.append(SkeletonRenderNode.from_bone(bone, color_by=color_by))
        return nodes

    def _connection_pairs(self) -> List[Tuple[str, str]]:
        pairs: List[Tuple[str, str]] = []
        name_map = {b.name: b for b in self.bones.values()}
        for bone in self.bones.values():
            for art in bone.articulations:
                other = name_map.get(art.get('bone'))
                if other:
                    pairs.append((bone.unique_id, other.unique_id))
        return pairs

    def visualize(
        self,
        mode: str = '3d_cylinders',
        color_by: str = 'material',
        interactivity: bool = True,
        output_target: str = 'ipython',
        **vis_kwargs,
    ) -> None:
        from .visualization import SkeletonVisualizer2D, SkeletonVisualizer3D

        nodes = self.to_render_nodes(color_by=color_by)
        connections = self._connection_pairs()

        if mode == '2d_graph':
            viz = SkeletonVisualizer2D()
            viz.render_graph(nodes, connections, **vis_kwargs)
        elif mode == '3d_points':
            viz = SkeletonVisualizer3D()
            viz.render_points(nodes, **vis_kwargs)
        else:  # default to 3d_cylinders
            viz = SkeletonVisualizer3D()
            viz.render_cylinders(nodes, connections, **vis_kwargs)
