from __future__ import annotations
from typing import List
from .assembler import AssemblerAgent

class VisualizerAgent:
    def __init__(self, assembler: AssemblerAgent):
        self.skeleton = assembler

    def render_ascii(self) -> str:
        tree = self.skeleton.hierarchy
        # find roots
        roots = [n for n in tree.keys() if not any(n in d.get("children", []) for d in tree.values())]
        lines: List[str] = []
        def walk(node: str, depth: int = 0):
            lines.append("  "*depth + f"- {node}")
            for c in tree.get(node, {}).get("children", []):
                walk(c, depth+1)
        for r in roots:
            walk(r)
        return "\n".join(lines) or "(empty)"

    def export_urdf(self) -> str:
        # Minimal URDF from connections; assumes identity inertials for brevity
        from xml.sax.saxutils import escape
        links = []
        joints = []
        for bone in self.skeleton.bones.values():
            links.append(f"<link name=\"{escape(bone.name)}\"/>")
            parent = bone.connections.get("parent")
            if parent:
                joints.append(
                    f"<joint name=\"{escape(parent+'_'+bone.name)}\" type=\"fixed\">"
                    f"<parent link=\"{escape(parent)}\"/><child link=\"{escape(bone.name)}\"/>"
                    f"</joint>"
                )
        return "<robot name=\"phy_skeleton\">" + "".join(links + joints) + "</robot>"

    def show_tf_tree(self) -> str:
        # Alias to ASCII for CLI preview
        return self.render_ascii()
