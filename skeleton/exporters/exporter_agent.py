from __future__ import annotations

from dataclasses import dataclass
from copy import deepcopy
import hashlib
import os
from pathlib import Path
from typing import Dict, Any, List
import json

from skeleton.field import SkeletonField
from skeleton.validation.validator_agent import ValidatorAgent, describe, nonfinite_paths
from .identity import ExportIdentity
from .publication import PublicationError, publish_files


def json_bytes(value: Any) -> bytes:
    """Strict, deterministic JSON; unknown None values remain JSON null."""
    for path, invalid in nonfinite_paths(value, "$"):
        raise ValueError(f"Nonfinite number at {path}: {describe(invalid)}")
    return (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n").encode("utf-8")


def sha256(content: bytes) -> str:
    return hashlib.sha256(content).hexdigest()


class ExportError(ValueError):
    """An unsuccessful attempt, with JSON-safe diagnostics and no success paths."""

    def __init__(self, code: str, report: Dict[str, Any]):
        super().__init__(code)
        self.code = code
        self.report = report


@dataclass
class ExporterAgent:
    """Strict review/validated record packages; not physical fabrication release."""

    skeleton: SkeletonField

    def export_all(self, dist_dir: Path = Path("dist"), reports_dir: Path = Path("reports"),
                   exports_dir: Path = Path("exports"), *, mode: str = "review") -> Dict[str, str]:
        """Publish ten files after validation, conversion and strict serialization.

        Review permits incomplete records and unresolved references. Validated
        requires both record and identity gates. Every mode requires valid,
        unique IDs and strict JSON. API review default preserves old callers;
        command-line entrypoints default to validated mode.
        """
        if mode not in ("review", "validated"):
            raise ValueError("Export mode must be review or validated")
        record_report = None
        identity_report = None

        def fail(code, issues=None, **extra):
            return ExportError(code, {
                "mode": mode, "published": False, "code": code,
                "record_validation": record_report, "identity_validation": identity_report,
                "issues": issues or [], **extra,
            })

        try:
            snapshot = deepcopy(self.skeleton)
            record_report = ValidatorAgent(snapshot).run()
            records = []
            conversion_errors = []
            for bone in sorted(snapshot.bones.values(), key=lambda b: str(b.unique_id)):
                try:
                    records.append(bone.to_fabrication_record())
                except (TypeError, ValueError, OverflowError, AttributeError, KeyError, RecursionError) as error:
                    conversion_errors.append({"bone_id": str(bone.unique_id), "issue": str(error)})
            if conversion_errors:
                raise fail("record_conversion_failed", conversion_errors)
            identity = ExportIdentity(records)
            identity_report = identity.report
        except ExportError:
            raise
        except (TypeError, ValueError, OverflowError, AttributeError, KeyError, RecursionError) as error:
            raise fail("invalid_input", [{"issue": str(error)}]) from error

        if identity.fatal:
            raise fail("invalid_identity")
        ready = record_report["summary"]["pass"] and identity_report["ready"]
        if mode == "validated" and not ready:
            raise fail("validation_failed")

        exported_records = deepcopy(records)
        for record in exported_records:
            uid = record["unique_id"]
            record["source_connections"] = record["connections"]
            record["connection_resolution"] = {
                "parent": identity.parent_references[uid], "children": identity.adjacency[uid],
            }
            record["connections"] = {
                "parent": identity.parents[uid],
                "children": sorted({row["bone_id"] for row in identity.adjacency[uid] if row["bone_id"] is not None}),
                "role": "anatomical_references",
            }

        export_report = {
            "mode": mode, "validated_ready": ready,
            "validation_scope": "canonical_records_and_export_identity",
            "record_validation_pass": record_report["summary"]["pass"],
            "record_issues": record_report["export_readiness"]["issues"],
            "identity_validation": identity_report,
            "fabrication_released": False,
        }

        paths = {
            "canonical_json": Path(dist_dir) / "skeleton_canonical.json",
            "tf_tree": Path(dist_dir) / "ros_tf_tree.json",
            "urdf_like": Path(dist_dir) / "skeleton_urdf_like.json",
            "bom": Path(exports_dir) / "fabrication_bom.json",
            "material_table": Path(exports_dir) / "material_table.json",
            "joint_table": Path(exports_dir) / "joint_table.json",
            "reference_audit": Path(reports_dir) / "reference_audit.json",
            "validation_report": Path(reports_dir) / "validation_report.json",
            "export_report": Path(reports_dir) / "export_report.json",
            "manifest": Path(dist_dir) / "export_manifest.json",
        }
        try:
            artifacts = {
                "canonical_json": exported_records, "tf_tree": self._tf_tree(records, identity),
                "urdf_like": self._urdf_like(records, identity), "bom": self._bom(records),
                "material_table": self._material_table(records), "joint_table": self._joint_table(records),
                "reference_audit": self._reference_audit(records),
                "validation_report": record_report, "export_report": export_report,
            }
            payloads = {paths[key]: json_bytes(value) for key, value in artifacts.items()}
            datasets = {}
            for bone in snapshot.bones.values():
                if bone.dataset is not None:
                    digest = sha256(json_bytes(bone.dataset))
                    datasets.setdefault(digest, []).append(bone.unique_id)
            schema_path = Path(__file__).parents[1] / "schema" / "bone.schema.json"
            manifest = {
                "package_version": "1.0.0", "mode": mode, "record_count": len(records),
                "validated_ready": ready, "fabrication_released": False,
                "validation_scope": export_report["validation_scope"],
                "source": {
                    "model": "BoneSpec", "source_records_sha256": sha256(json_bytes(records)),
                    "schema": {"path": "skeleton/schema/bone.schema.json", "sha256": sha256(schema_path.read_bytes())},
                    "datasets": [{"sha256": digest, "bone_ids": sorted(ids)} for digest, ids in sorted(datasets.items())],
                    "revisions": {record["unique_id"]: record["revision"] for record in records},
                },
                "files": {
                    key: {"path": Path(os.path.relpath(path, paths["manifest"].parent)).as_posix(),
                          "sha256": sha256(payloads[path])}
                    for key, path in paths.items() if key != "manifest"
                },
            }
            payloads[paths["manifest"]] = json_bytes(manifest)
        except (TypeError, ValueError, OverflowError, AttributeError, KeyError, RecursionError, OSError) as error:
            raise fail("serialization_failed", [{"issue": str(error)}]) from error
        try:
            publish_files(payloads, paths["manifest"])
        except PublicationError as error:
            raise fail("publication_failed", [{"issue": str(error)}],
                       previous_package_restored=not error.rollback_errors,
                       rollback_errors=error.rollback_errors, recovery_files=error.recovery_files) from error
        return {key: str(path) for key, path in paths.items()}

    def _tf_tree(self, records: List[Dict[str, Any]], identity: ExportIdentity) -> List[Dict[str, Any]]:
        tree = []
        for rec in records:
            uid = rec["unique_id"]
            tree.append({
                "frame_id": uid, "display_name": rec["name"],
                "parent": identity.parents[uid], "children": identity.children[uid],
                "parent_reference": identity.parent_references[uid],
                "relationship_role": "anatomical_parent_reference",
                "origin_mm": None, "rotation_rpy_rad": None,
                "transform_status": "unresolved",
            })
        return tree

    def _urdf_like(self, records: List[Dict[str, Any]], identity: ExportIdentity) -> Dict[str, Any]:
        links = []
        joints = []
        for rec in records:
            links.append({
                "name": rec["unique_id"], "display_name": rec["name"],
                "inertial": rec.get("physics", {}),
                "geometry": rec.get("geometry", {}),
            })
            for index, ji in enumerate(rec.get("joint_interfaces", [])):
                joints.append({
                    "name": f"{rec['unique_id']}__joint_{index}", "display_name": ji.get("name"),
                    "parent": identity.parents[rec["unique_id"]], "child": rec["unique_id"],
                    "type": ji.get("type"), "definition": ji,
                    "definition_status": "unqualified",
                })
        return {"robot": "phy_skeleton", "role": "review_inventory", "mechanical_transforms_validated": False,
                "links": links, "joints": joints}

    def _bom(self, records: List[Dict[str, Any]]) -> Dict[str, Any]:
        items = []
        for rec in records:
            items.append({
                "part_number": rec.get("unique_id"),
                "name": rec["name"],
                "material": rec.get("material", {}).get("name", "bone"),
                "revision": rec.get("revision"),
                "tolerance": rec.get("tolerance", {}),
            })
        return {"count": len(items), "items": items}

    def _material_table(self, records: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        return [{
            "bone": rec["unique_id"], "display_name": rec["name"],
            "material": rec.get("material", {}).get("name", "bone"),
            "density": rec.get("material", {}).get("density"),
        } for rec in records]

    def _joint_table(self, records: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        table = []
        for rec in records:
            for index, joint in enumerate(rec.get("joint_interfaces", [])):
                table.append({
                    "bone": rec["unique_id"], "display_name": rec["name"],
                    "joint": f"{rec['unique_id']}__joint_{index}", "joint_display_name": joint.get("name"),
                    "type": joint.get("type"),
                    "mount_point": joint.get("mount_point"),
                })
        return table

    def _reference_audit(self, records: List[Dict[str, Any]]) -> Dict[str, Any]:
        missing = [rec["unique_id"] for rec in records if not rec.get("references")]
        return {
            "total_bones": len(records),
            "with_references": len(records) - len(missing),
            "missing_references": missing,
        }
