from __future__ import annotations

import json
from pathlib import Path
from typing import Dict

import pandas as pd
from skeleton.field import SkeletonField


class ReportAgent:
    """Generate CSV, JSON and Markdown validation reports."""

    def __init__(self, skeleton: SkeletonField, results: Dict[str, object]):
        self.skeleton = skeleton
        self.results = results

    def export(self, directory: str) -> None:
        path = Path(directory)
        path.mkdir(parents=True, exist_ok=True)

        rows = []
        for bone in self.skeleton.bones.values():
            row = {
                "id": bone.unique_id,
                "name": bone.name,
                "material": bone.material.get("name"),
            }
            row.update(bone.dimensions)
            row["density"] = bone.material.get("density")
            row["mass_g"] = bone.material.get("mass_g")
            rows.append(row)
        df = pd.DataFrame(rows)
        df.to_csv(path / "bones_metrics.csv", index=False)
        df.to_json(path / "bones_metrics.json", orient="records", indent=2)

        with open(path / "validation_results.json", "w", encoding="utf-8") as fh:
            json.dump(self.results, fh, indent=2)

        md_lines = ["# Validation Summary", "", f"**Pass:** {self.results['summary']['pass']}", f"**Warnings:** {self.results['summary']['warnings']}", ""]
        if self.results.get("missing_metrics"):
            md_lines.append("## Missing Metrics")
            for uid, key in self.results["missing_metrics"]:
                md_lines.append(f"- {uid}: {key}")
            md_lines.append("")
        if self.results.get("out_of_range"):
            md_lines.append("## Volume Discrepancies (>7%)")
            for uid, diff in self.results["out_of_range"].items():
                md_lines.append(f"- {uid}: {diff:.2%}")
            md_lines.append("")
        if self.results.get("material_fail"):
            md_lines.append("## Material Issues")
            for item in self.results["material_fail"]:
                md_lines.append(f"- {item}")
            md_lines.append("")
        with open(path / "validation_summary.md", "w", encoding="utf-8") as fh:
            fh.write("\n".join(md_lines))

        # Optional HTML for quick view
        html_file = path / "bones_metrics.html"
        df.to_html(html_file, index=False)
