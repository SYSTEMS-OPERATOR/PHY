from __future__ import annotations
from typing import Dict, List
import yaml, csv, pathlib

ROOT = pathlib.Path(__file__).resolve().parents[1]
INDEX = ROOT / "references" / "index.yml"

class ReferenceAgent:
    def __init__(self):
        self.references: Dict[str, List[str]] = {}
        if INDEX.exists():
            self.references = yaml.safe_load(INDEX.read_text()) or {}

    def validate_references(self, bone_agent) -> List[str]:
        refs = bone_agent.get_reference() or self.references.get(bone_agent.name, [])
        missing = [] if refs else [bone_agent.name]
        return missing

    def export(self, fmt: str = "csv") -> str:
        if fmt == "csv":
            out = ROOT / "references.csv"
            with out.open("w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["bone", "reference"])
                for bone, refs in sorted(self.references.items()):
                    for r in refs:
                        w.writerow([bone, r])
            return str(out)
        if fmt == "md":
            out = ROOT / "REFERENCES.md"
            lines = ["# References", ""]
            for bone, refs in sorted(self.references.items()):
                lines.append(f"- **{bone}**")
                for r in refs:
                    lines.append(f"  - {r}")
            out.write_text("\n".join(lines))
            return str(out)
        raise ValueError("unsupported format")
