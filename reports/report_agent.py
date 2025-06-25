from __future__ import annotations

import json
from pathlib import Path
from typing import Dict
import pandas as pd

from skeleton.field import SkeletonField


class ReportAgent:
    """Generate validation reports."""

    def __init__(self, skeleton: SkeletonField, results: Dict[str, object]):
        self.skeleton = skeleton
        self.results = results

    def export(self, out_dir: str) -> None:
        out_path = Path(out_dir)
        out_path.mkdir(parents=True, exist_ok=True)
        self._export_csv(out_path)
        self._export_json(out_path)
        self._export_markdown(out_path)

    def _export_csv(self, out_path: Path) -> None:
        rows = []
        for b in self.skeleton.bones.values():
            rows.append({
                "name": b.name,
                "uid": b.unique_id,
                "material": b.material.get("name"),
                "density": b.material.get("density"),
                "length_cm": b.dimensions.get("length_cm"),
                "width_cm": b.dimensions.get("width_cm"),
                "thickness_cm": b.dimensions.get("thickness_cm"),
            })
        df = pd.DataFrame(rows)
        df.to_csv(out_path / "bones_metrics.csv", index=False)
        df.to_html(out_path / "bones_metrics.html", index=False)

    def _export_json(self, out_path: Path) -> None:
        with open(out_path / "validation_results.json", "w", encoding="utf-8") as fh:
            json.dump(self.results, fh, indent=2)

    def _export_markdown(self, out_path: Path) -> None:
        lines = ["# Validation Summary", ""]
        summary = self.results.get("summary", {})
        lines.append(f"**Pass:** {summary.get('pass')}  ")
        lines.append(f"**Warnings:** {summary.get('warnings')}  ")
        lines.append("")
        lines.append("## Missing Metrics")
        missing = self.results.get("missing_metrics")
        if missing:
            for m in missing:
                lines.append(f"- {m}")
        else:
            lines.append("None")
        lines.append("")
        lines.append("## Out of Range")
        oor = self.results.get("out_of_range")
        if oor:
            lines.append("| Bone | Δ |")
            lines.append("|------|---|")
            for bone, delta in oor.items():
                lines.append(f"| {bone} | {delta:.3f} |")
        else:
            lines.append("None")
        lines.append("")
        lines.append("## Material Fail")
        mf = self.results.get("material_fail")
        if mf:
            for m in mf:
                lines.append(f"- {m}")
        else:
            lines.append("None")
        with open(out_path / "validation_summary.md", "w", encoding="utf-8") as fh:
            fh.write("\n".join(lines))
