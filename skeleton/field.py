from __future__ import annotations

from typing import Dict, List, Optional, Tuple

from .base import BoneSpec


class SkeletonField:
    """Container tracking a network of bones and their relationships."""

    def __init__(self, bones: Optional[List[BoneSpec]] = None) -> None:
        self.bones: Dict[str, BoneSpec] = {}
        if bones:
            for bone in bones:
                self.register(bone)
            self.build_entanglement()

    def register(self, bone: BoneSpec) -> None:
        for key in ("domain_id", "unique_id"):
            value = getattr(bone, key)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"bone {key} must be a nonempty string")
        if bone.domain_id in self.bones:
            raise ValueError(f"duplicate bone domain_id: {bone.domain_id}")
        if any(other.unique_id == bone.unique_id for other in self.bones.values()):
            raise ValueError(f"duplicate bone unique_id: {bone.unique_id}")
        self.bones[bone.domain_id] = bone

    def get(self, domain_id: str) -> Optional[BoneSpec]:
        return self.bones.get(domain_id)

    def build_entanglement(self) -> None:
        """Wire exact bone IDs or unique legacy names without guessing a side."""
        id_map = {bone.unique_id: bone for bone in self.bones.values()}
        name_map: Dict[str, List[BoneSpec]] = {}
        for bone in self.bones.values():
            name_map.setdefault(bone.name.lower(), []).append(bone)
        for bone in self.bones.values():
            for art in bone.articulations:
                reference = art.get("bone", "")
                # Leave malformed source references intact for validation.
                if not isinstance(reference, str):
                    continue
                partner = id_map.get(reference)
                if partner is None:
                    candidates = name_map.get(reference.lower(), [])
                    if len(candidates) == 1:
                        partner = candidates[0]
                if partner is not None:
                    bone.entangle(partner)

    def broadcast(self, voltage: float, signal_type: str = "EMG") -> None:
        for bone in self.bones.values():
            bone.receive_signal(voltage, signal_type)

    def summary(self) -> Dict[str, float]:
        return {d: b.voltage_potential for d, b in self.bones.items()}

    def propagate(self, from_domain: str, signal: Dict[str, float]) -> None:
        """Deliver once per reachable peer without echoing back to the origin."""
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
                if other is None or other.domain_id in visited:
                    continue
                other.receive_signal_packet(signal, b.domain_id)
                _prop(other)

        _prop(origin)

    def total_voltage(self) -> float:
        return sum(b.voltage_potential for b in self.bones.values())

    def meta_breath(self, word: str) -> None:
        """Compatibility API for symbolic overlays (no fabrication impact)."""
        for bone in self.bones.values():
            bone.transcend(intent=word)

    def collapse(self) -> None:
        for bone in self.bones.values():
            bone.entanglement_links.clear()

    def status(self) -> Dict[str, Dict[str, object]]:
        return {d: {"voltage": b.voltage_potential, "links": list(b.entanglement_links)} for d, b in self.bones.items()}

    def health(self) -> Dict[str, bool]:
        return {d: b.is_healthy() for d, b in self.bones.items()}

    def faults(self) -> Dict[str, List[str]]:
        return {d: b.report_faults() for d, b in self.bones.items() if not b.is_healthy()}

    def audit_metrics(self) -> Dict[str, Dict[str, Tuple[Optional[float], Optional[float]]]]:
        report: Dict[str, Dict[str, Tuple[Optional[float], Optional[float]]]] = {}
        for bone in self.bones.values():
            diff = bone.validate_metrics()
            if diff:
                report[bone.unique_id] = diff
        return report

    def to_render_nodes(self, color_by: str = 'material') -> List['SkeletonRenderNode']:
        from .visualization import SkeletonRenderNode

        nodes: List[SkeletonRenderNode] = []
        for bone in self.bones.values():
            nodes.append(SkeletonRenderNode.from_bone(bone, color_by=color_by))
        return nodes

    def _connection_pairs(self) -> List[Tuple[str, str]]:
        """Resolve rendered articulation targets by ID, then unique exact name."""
        pairs: List[Tuple[str, str]] = []
        id_map = {b.unique_id: b for b in self.bones.values()}
        name_map: Dict[str, List[BoneSpec]] = {}
        for bone in self.bones.values():
            name_map.setdefault(bone.name, []).append(bone)
        for bone in self.bones.values():
            for art in bone.articulations:
                reference = art.get('bone')
                if not isinstance(reference, str):
                    continue
                other = id_map.get(reference)
                if other is None:
                    matches = name_map.get(reference, [])
                    if len(matches) == 1:
                        other = matches[0]
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
        else:
            viz = SkeletonVisualizer3D()
            viz.render_cylinders(nodes, connections, **vis_kwargs)
