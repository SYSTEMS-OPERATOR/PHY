from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List

import pandas as pd

from skeleton.base import BoneSpec


class ReportAgent:
    """Generate CSV, JSON, and Markdown reports from validation results."""

    def __init__(self, bones: List[BoneSpec], results: Dict[str, Any]):
        self.bones = bones
        self.results = results

    def export(self, out_dir: str) -> None:
        path = Path(out_dir)
        path.mkdir(parents=True, exist_ok=True)
        self._export_csv(path / "bones_metrics.csv")
        self._export_json(path / "validation_results.json")
        self._export_markdown(path / "validation_summary.md")
        self._export_html(path / "bones_metrics.html")

    def _export_csv(self, file: Path) -> None:
        rows = []
        for b in self.bones:
            row = {
                "uid": b.unique_id,
                "name": b.name,
                "material": b.material.get("name"),
                "length_cm": b.dimensions.get("length_cm"),
                "width_cm": b.dimensions.get("width_cm"),
                "thickness_cm": b.dimensions.get("thickness_cm"),
                "mass_g": b.material.get("mass_g"),
                "density_kg_m3": b.material.get("density"),
            }
            rows.append(row)
        df = pd.DataFrame(rows)
        df.to_csv(file, index=False)

    def _export_json(self, file: Path) -> None:
        with open(file, "w", encoding="utf-8") as fh:
            json.dump(self.results, fh, indent=2)

    def _export_markdown(self, file: Path) -> None:
        lines = ["# Validation Summary", ""]
        lines.append(f"Warnings: {self.results['summary']['warnings']}")
        lines.append("")
        if self.results.get("missing_metrics"):
            lines.append("## Missing Metrics")
            for m in self.results["missing_metrics"]:
                lines.append(f"- {m}")
            lines.append("")
        if self.results.get("out_of_range"):
            lines.append("## Volume Discrepancies (>7%)")
            for k, v in self.results["out_of_range"].items():
                lines.append(f"- {k}: {v:.2%}")
            lines.append("")
        if self.results.get("material_fail"):
            lines.append("## Material Failures")
            for m in self.results["material_fail"]:
                lines.append(f"- {m}")
            lines.append("")
        with open(file, "w", encoding="utf-8") as fh:
            fh.write("\n".join(lines))

    def _export_html(self, file: Path) -> None:
        df = pd.read_csv(file.with_suffix(".csv"))
        df.to_html(file, index=False)

