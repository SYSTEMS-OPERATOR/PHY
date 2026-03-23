from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Any, Tuple
import json

from skeleton.field import SkeletonField


@dataclass
class ValidatorAgent:
    """Fabrication-grade validator for canonical bone records."""

    skeleton: SkeletonField

    def run(self) -> Dict[str, Any]:
        records = [b.to_fabrication_record() for b in self.skeleton.bones.values()]
        report: Dict[str, Any] = {
            "summary": {"pass": True, "total_bones": len(records), "failed_checks": 0},
            "missing_required_fields": [],
            "duplicate_bone_ids": [],
            "invalid_parent_child_references": [],
            "impossible_geometry_values": [],
            "invalid_unit_combinations": [],
            "material_incompleteness": [],
            "missing_mount_points": [],
            "missing_joint_interfaces": [],
            "missing_references": [],
            "mass_inertia_plausibility": [],
            "dataset_schema_mismatch": [],
            "export_readiness": {"ready": True, "issues": []},
        }

        required_fields = [
            "name", "latin_name", "bone_type", "region", "dimensions", "units", "geometry",
            "material", "physics", "connections", "joint_interfaces", "mount_points",
            "manufacturing_notes", "tolerance", "references", "revision", "source_ids",
        ]

        ids = set()
        valid_ids = {r.get("unique_id") for r in records}

        for rec in records:
            uid = rec.get("unique_id", "unknown")

            for field_name in required_fields:
                value = rec.get(field_name)
                if value is None or value == "" or value == []:
                    report["missing_required_fields"].append({"bone": uid, "field": field_name})

            if uid in ids:
                report["duplicate_bone_ids"].append(uid)
            ids.add(uid)

            parent = (rec.get("connections") or {}).get("parent")
            children = (rec.get("connections") or {}).get("children") or []
            for child in children:
                if child and child not in {r["name"] for r in records} and child not in valid_ids:
                    report["invalid_parent_child_references"].append({"bone": uid, "child": child})
            if parent and parent not in {r["name"] for r in records} and parent not in valid_ids:
                report["invalid_parent_child_references"].append({"bone": uid, "parent": parent})

            dims = rec.get("dimensions", {})
            for dname, dval in dims.items():
                if isinstance(dval, (int, float)) and dval <= 0:
                    report["impossible_geometry_values"].append({"bone": uid, "dimension": dname, "value": dval})

            units = rec.get("units", {})
            if units.get("length") != "mm" or units.get("mass") != "kg" or units.get("density") != "kg/m^3":
                report["invalid_unit_combinations"].append({"bone": uid, "units": units})

            material = rec.get("material", {})
            if "density" not in material:
                report["material_incompleteness"].append({"bone": uid, "missing": "density"})

            if not rec.get("mount_points"):
                report["missing_mount_points"].append(uid)
            if not rec.get("joint_interfaces"):
                report["missing_joint_interfaces"].append(uid)
            if not rec.get("references"):
                report["missing_references"].append(uid)

            mass = (rec.get("physics") or {}).get("mass_kg")
            if mass is not None and (mass <= 0 or mass > 100):
                report["mass_inertia_plausibility"].append({"bone": uid, "mass_kg": mass})

            if rec.get("dataset_key") is None and "dataset_key" in rec:
                report["dataset_schema_mismatch"].append({"bone": uid, "issue": "missing dataset_key"})

        for key, value in report.items():
            if key in {"summary", "export_readiness"}:
                continue
            if value:
                report["summary"]["failed_checks"] += 1

        report["summary"]["pass"] = report["summary"]["failed_checks"] == 0
        report["export_readiness"]["ready"] = report["summary"]["pass"]
        if not report["summary"]["pass"]:
            report["export_readiness"]["issues"].append("Validation failures detected")
        return report

    @staticmethod
    def write_reports(report: Dict[str, Any], out_dir: Path = Path("reports")) -> Tuple[Path, Path]:
        out_dir.mkdir(parents=True, exist_ok=True)
        json_path = out_dir / "validation_report.json"
        md_path = out_dir / "validation_report.md"
        json_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

        lines: List[str] = [
            "# Fabrication Validation Report",
            "",
            f"- Pass: **{report['summary']['pass']}**",
            f"- Total bones: **{report['summary']['total_bones']}**",
            f"- Failed check categories: **{report['summary']['failed_checks']}**",
            "",
        ]
        for key, value in report.items():
            if key in {"summary", "export_readiness"}:
                continue
            lines.append(f"## {key.replace('_', ' ').title()}")
            if not value:
                lines.append("- None")
            elif isinstance(value, list):
                for row in value[:25]:
                    lines.append(f"- {row}")
                if len(value) > 25:
                    lines.append(f"- ... ({len(value) - 25} additional)")
            else:
                lines.append(f"- {value}")
            lines.append("")
        md_path.write_text("\n".join(lines), encoding="utf-8")
        return json_path, md_path
