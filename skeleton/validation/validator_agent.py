from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Any, Tuple
import json
import math

from skeleton.base import CANONICAL_UNITS
from skeleton.field import SkeletonField


TEXT_FIELDS = ("name", "bone_type", "region", "revision", "unique_id")
MAPPING_FIELDS = ("dimensions", "units", "geometry", "material", "physics", "connections", "tolerance")
TEXT_LIST_FIELDS = ("manufacturing_notes", "references", "source_ids")
INTERFACE_FIELDS = {"mount_points": "missing_mount_points", "joint_interfaces": "missing_joint_interfaces"}


def finite_number(value: Any) -> bool:
    """Do not coerce booleans, numeric strings, or overflowing integers."""
    if type(value) not in (int, float):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def describe(value: Any) -> str:
    """JSON-safe diagnostics, including NaN/Infinity and malformed values."""
    try:
        return repr(value)[:160]
    except (ValueError, OverflowError):
        return f"<{type(value).__name__}>"


def nonfinite_paths(value: Any, path: str):
    if type(value) in (int, float) and not finite_number(value):
        yield path, value
    elif isinstance(value, dict):
        for key in sorted(value, key=str):
            yield from nonfinite_paths(value[key], f"{path}.{key}")
    elif isinstance(value, (list, tuple)):
        for index, item in enumerate(value):
            yield from nonfinite_paths(item, f"{path}[{index}]")


@dataclass
class ValidatorAgent:
    """Fabrication-grade validator for canonical bone records."""

    skeleton: SkeletonField

    def run(self) -> Dict[str, Any]:
        bones = sorted(self.skeleton.bones.values(), key=lambda b: str(b.unique_id))
        report: Dict[str, Any] = {
            "summary": {"pass": True, "total_bones": len(bones), "failed_checks": 0},
            "empty_skeleton": [] if bones else ["No bones were provided"],
            "missing_required_fields": [],
            "invalid_field_types": [],
            "record_conversion_errors": [],
            "missing_dimension_values": [],
            "duplicate_bone_ids": [],
            "invalid_parent_child_references": [],
            "impossible_geometry_values": [],
            "invalid_unit_combinations": [],
            "material_incompleteness": [],
            "missing_mount_points": [],
            "missing_joint_interfaces": [],
            "missing_references": [],
            "mass_inertia_plausibility": [],
            "invalid_tolerances": [],
            "dataset_schema_mismatch": [],
            "export_readiness": {"ready": True, "issues": []},
        }

        ids = set()
        valid_ids = {b.unique_id for b in bones if isinstance(b.unique_id, str)}
        valid_names = {b.name for b in bones if isinstance(b.name, str)}

        for bone in bones:
            uid = bone.unique_id if isinstance(bone.unique_id, str) else describe(bone.unique_id)
            if uid in ids:
                report["duplicate_bone_ids"].append(uid)
            ids.add(uid)

            # Validate raw types before normalization can coerce strings/bools,
            # and before malformed containers can throw from BoneSpec export.
            bad_container = False
            for field_name in TEXT_FIELDS:
                value = getattr(bone, field_name)
                if not isinstance(value, str) or not value.strip():
                    report["missing_required_fields"].append({"bone": uid, "field": field_name})
            if bone.latin_name is not None and (not isinstance(bone.latin_name, str) or not bone.latin_name.strip()):
                report["invalid_field_types"].append({"bone": uid, "field": "latin_name", "issue": "expected nonempty string or legacy name fallback"})
            for field_name in MAPPING_FIELDS:
                value = getattr(bone, field_name)
                if not isinstance(value, dict) or any(not isinstance(k, str) or not k.strip() for k in value):
                    report["invalid_field_types"].append({"bone": uid, "field": field_name, "issue": "expected object with nonempty string keys"})
                    bad_container = True
                elif not value and field_name in ("geometry", "tolerance"):
                    report["missing_required_fields"].append({"bone": uid, "field": field_name})
            for field_name in TEXT_LIST_FIELDS:
                value = getattr(bone, field_name)
                if not isinstance(value, list) or any(not isinstance(v, str) or not v.strip() for v in value):
                    report["invalid_field_types"].append({"bone": uid, "field": field_name, "issue": "expected array of nonempty strings"})
                    bad_container = True
                elif not value:
                    report["missing_required_fields"].append({"bone": uid, "field": field_name})
            for field_name, category in INTERFACE_FIELDS.items():
                value = getattr(bone, field_name)
                if not isinstance(value, list) or not value or any(not isinstance(v, dict) or not v for v in value):
                    report[category].append(uid)
                if not isinstance(value, list):
                    bad_container = True
                elif not value:
                    report["missing_required_fields"].append({"bone": uid, "field": field_name})
            if not isinstance(bone.references, list) or not bone.references:
                report["missing_references"].append(uid)
            if bad_container:
                continue

            dims = bone.dimensions
            if not dims:
                report["missing_dimension_values"].append({"bone": uid, "dimension": "*", "issue": "no dimensions provided"})
            for dname, dval in dims.items():
                if dval is None:
                    report["missing_dimension_values"].append({"bone": uid, "dimension": dname, "issue": "unknown measurement"})
                elif not finite_number(dval) or dval <= 0:
                    report["impossible_geometry_values"].append({"bone": uid, "dimension": dname, "value": describe(dval)})
            for unit, expected in CANONICAL_UNITS.items():
                if bone.units.get(unit) != expected:
                    report["invalid_unit_combinations"].append({"bone": uid, "unit": unit, "expected": expected, "actual": describe(bone.units.get(unit))})
            density = bone.material.get("density")
            if not finite_number(density) or density <= 0:
                report["material_incompleteness"].append({"bone": uid, "field": "density", "value": describe(density)})
            for key, value in bone.tolerance.items():
                if not finite_number(value) or value < 0:
                    report["invalid_tolerances"].append({"bone": uid, "field": key, "value": describe(value)})
            for field_name in ("geometry", "joint_interfaces", "mount_points", "physics"):
                category = "impossible_geometry_values" if field_name == "geometry" else "invalid_field_types"
                for path, value in nonfinite_paths(getattr(bone, field_name), field_name):
                    report[category].append({"bone": uid, "field": path, "value": describe(value)})

            parent = bone.connections.get("parent")
            if parent not in (None, "") and (not isinstance(parent, str) or parent not in valid_ids | valid_names):
                report["invalid_parent_child_references"].append({"bone": uid, "parent": describe(parent)})
            children = bone.connections.get("children", [])
            if not isinstance(children, list):
                report["invalid_parent_child_references"].append({"bone": uid, "field": "children", "issue": "expected array of bone references"})
            else:
                for child in children:
                    if not isinstance(child, str) or child not in valid_ids | valid_names:
                        report["invalid_parent_child_references"].append({"bone": uid, "child": describe(child)})

            if bone.dataset is not None:
                if not isinstance(bone.dataset, dict):
                    report["dataset_schema_mismatch"].append({"bone": uid, "issue": "expected dataset object"})
                elif not isinstance(bone.dataset_key, str) or bone.dataset_key not in bone.dataset:
                    report["dataset_schema_mismatch"].append({"bone": uid, "issue": "missing or unknown dataset_key"})

            try:
                rec = bone.to_fabrication_record()
            except (TypeError, ValueError, OverflowError, AttributeError) as exc:
                report["record_conversion_errors"].append({"bone": uid, "error": type(exc).__name__, "issue": str(exc)})
                continue
            # cm conversion or volume multiplication can overflow even when
            # each original number is finite.
            for key, value in rec["dimensions"].items():
                if value is not None and not finite_number(value) and finite_number(dims.get(key, dims.get(key.replace("_mm", "_cm")))):
                    report["impossible_geometry_values"].append({"bone": uid, "dimension": key, "issue": "nonfinite normalized dimension"})
            mass = rec["physics"].get("mass_kg")
            if not finite_number(mass) or not 0 < mass <= 100:
                report["mass_inertia_plausibility"].append({"bone": uid, "mass_kg": describe(mass)})

        for key, value in report.items():
            if key in {"summary", "export_readiness"}:
                continue
            if value:
                report["summary"]["failed_checks"] += 1

        report["summary"]["pass"] = report["summary"]["failed_checks"] == 0
        report["export_readiness"]["ready"] = report["summary"]["pass"]
        report["export_readiness"]["issues"] = [key for key, value in report.items() if isinstance(value, list) and value]
        return report

    @staticmethod
    def write_reports(report: Dict[str, Any], out_dir: Path = Path("reports")) -> Tuple[Path, Path]:
        out_dir.mkdir(parents=True, exist_ok=True)
        json_path = out_dir / "validation_report.json"
        md_path = out_dir / "validation_report.md"
        json_path.write_text(json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")

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
