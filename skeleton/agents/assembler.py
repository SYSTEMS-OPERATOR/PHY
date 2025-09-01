from __future__ import annotations
from typing import Dict, Any, List
from dataclasses import dataclass, field
import yaml, json, pathlib
from .bone import BoneAgent

ROOT = pathlib.Path(__file__).resolve().parents[2]
BONES_DIR = ROOT / "skeleton" / "bones"

@dataclass
class AssemblerAgent:
    bones: Dict[str, BoneAgent] = field(default_factory=dict)
    hierarchy: Dict[str, Any] = field(default_factory=dict)

    def discover_bones(self, path: pathlib.Path = BONES_DIR) -> None:
        for yml in path.rglob("*.yml"):
            data = yaml.safe_load(yml.read_text())
            bone = BoneAgent(**data)
            self.bones[bone.name] = bone

    def assemble(self) -> None:
        # Build hierarchy from connections
        graph = {}
        for b in self.bones.values():
            p = b.connections.get("parent")
            graph.setdefault(b.name, {"children": []})
            if p:
                graph.setdefault(p, {"children": []})
                graph[p]["children"].append(b.name)
        self.hierarchy = graph

    def set_global_material(self, material_props: Dict[str, float]) -> None:
        for b in self.bones.values():
            b.set_material(material_props)

    def export_ros_tf(self) -> List[Dict[str, Any]]:
        return [b.as_ros_node() for b in self.bones.values()]

    def validate(self) -> Dict[str, Any]:
        report = {"missing": [], "orphans": [], "ok": True}
        # Optional: check against canonical list if provided
        # Here we flag nodes with no parent and not declared as root
        roots = [n for n,v in self.hierarchy.items() if not any(n in d.get("children", []) for d in self.hierarchy.values())]
        if len(roots) == 0:
            report["orphans"].append("no_root")
            report["ok"] = False
        return report

    # helpers
    def to_json(self) -> str:
        payload = {
            "bones": {k: {
                "dimensions": v.dimensions,
                "material": v.material,
                "physics": v.physics,
                "connections": v.connections,
            } for k,v in self.bones.items()},
            "hierarchy": self.hierarchy,
        }
        return json.dumps(payload, indent=2)
