from __future__ import annotations
from dataclasses import dataclass, field
from typing import Dict, Any, List, Optional
import math

@dataclass
class BoneAgent:
    name: str
    latin_name: Optional[str] = None
    type: Optional[str] = None
    region: Optional[str] = None
    dimensions: Dict[str, float] = field(default_factory=dict)
    geometry: Dict[str, Any] = field(default_factory=dict)
    material: Dict[str, float] = field(default_factory=lambda: {"density": 1850.0, "E": 17e9})
    physics: Dict[str, float] = field(default_factory=dict)
    connections: Dict[str, Any] = field(default_factory=dict)  # {"parent": ..., "children": [...], "joints": {...}}
    references: List[str] = field(default_factory=list)

    def set_material(self, material_props: Dict[str, float]) -> None:
        self.material.update(material_props)
        self._recalc_physics()

    def export_geometry(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "node_grid": self.geometry.get("node_grid", []),
            "faces": self.geometry.get("faces", []),
            "landmarks": self.geometry.get("landmarks", {}),
            "bbox": self.geometry.get("bbox", self._bbox_from_dimensions()),
        }

    def get_reference(self) -> List[str]:
        return list(self.references)

    def as_ros_node(self) -> Dict[str, Any]:
        # Minimal TF-ish description; consumers can publish frames
        return {
            "frame_id": self.name,
            "parent": self.connections.get("parent"),
            "children": self.connections.get("children", []),
            "origin": self.geometry.get("origin", [0.0, 0.0, 0.0]),
            "orientation": self.geometry.get("orientation", [0.0, 0.0, 0.0, 1.0]),
        }

    # --- internals ---
    def _bbox_from_dimensions(self):
        L = self.dimensions.get("length", 0.0)
        W = self.dimensions.get("width", 0.0)
        T = self.dimensions.get("thickness", 0.0)
        return [L, W, T]

    def _approx_volume(self) -> float:
        # Conservative rectangular prism approximation (grid-native)
        L, W, T = self._bbox_from_dimensions()
        vol = max(L, 0.0) * max(W, 0.0) * max(T, 0.0)
        return float(vol)

    def _recalc_physics(self) -> None:
        rho = float(self.material.get("density", 1850.0))  # kg/m^3
        vol = self._approx_volume()  # m^3
        mass = rho * vol
        L = self.dimensions.get("length", 0.0)
        # Thin rod about center: I = (1/12) m L^2 (axis simplification)
        I = (1.0/12.0) * mass * (L ** 2)
        self.physics.update({"mass": mass, "I_center": I})
