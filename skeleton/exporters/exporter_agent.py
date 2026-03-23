from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Any, List
import json

from skeleton.field import SkeletonField


@dataclass
class ExporterAgent:
    """Deterministic exporter for fabrication and robotics artifacts."""

    skeleton: SkeletonField

    def export_all(self, dist_dir: Path = Path("dist"), reports_dir: Path = Path("reports"), exports_dir: Path = Path("exports")) -> Dict[str, str]:
        dist_dir.mkdir(parents=True, exist_ok=True)
        reports_dir.mkdir(parents=True, exist_ok=True)
        exports_dir.mkdir(parents=True, exist_ok=True)

        records = [b.to_fabrication_record() for b in self.skeleton.bones.values()]
        records = sorted(records, key=lambda r: r.get("unique_id", r["name"]))

        paths = {
            "canonical_json": str(dist_dir / "skeleton_canonical.json"),
            "tf_tree": str(dist_dir / "ros_tf_tree.json"),
            "urdf_like": str(dist_dir / "skeleton_urdf_like.json"),
            "bom": str(exports_dir / "fabrication_bom.json"),
            "material_table": str(exports_dir / "material_table.json"),
            "joint_table": str(exports_dir / "joint_table.json"),
            "reference_audit": str(reports_dir / "reference_audit.json"),
        }

        Path(paths["canonical_json"]).write_text(json.dumps(records, indent=2), encoding="utf-8")
        Path(paths["tf_tree"]).write_text(json.dumps(self._tf_tree(records), indent=2), encoding="utf-8")
        Path(paths["urdf_like"]).write_text(json.dumps(self._urdf_like(records), indent=2), encoding="utf-8")
        Path(paths["bom"]).write_text(json.dumps(self._bom(records), indent=2), encoding="utf-8")
        Path(paths["material_table"]).write_text(json.dumps(self._material_table(records), indent=2), encoding="utf-8")
        Path(paths["joint_table"]).write_text(json.dumps(self._joint_table(records), indent=2), encoding="utf-8")
        Path(paths["reference_audit"]).write_text(json.dumps(self._reference_audit(records), indent=2), encoding="utf-8")
        return paths

    def _tf_tree(self, records: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        tree = []
        for rec in records:
            connections = rec.get("connections", {})
            tree.append({
                "frame_id": rec["name"],
                "parent": connections.get("parent"),
                "children": connections.get("children", []),
                "origin": rec.get("geometry", {}).get("origin_mm", [0, 0, 0]),
            })
        return tree

    def _urdf_like(self, records: List[Dict[str, Any]]) -> Dict[str, Any]:
        links = []
        joints = []
        for rec in records:
            links.append({
                "name": rec["name"],
                "inertial": rec.get("physics", {}),
                "geometry": rec.get("geometry", {}),
            })
            for ji in rec.get("joint_interfaces", []):
                joints.append({
                    "name": ji.get("name", f"{rec['name']}_joint"),
                    "parent": rec.get("connections", {}).get("parent"),
                    "child": rec["name"],
                    "type": ji.get("type", "fixed"),
                })
        return {"robot": "phy_skeleton", "links": links, "joints": joints}

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
            "bone": rec["name"],
            "material": rec.get("material", {}).get("name", "bone"),
            "density": rec.get("material", {}).get("density"),
        } for rec in records]

    def _joint_table(self, records: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        table = []
        for rec in records:
            for joint in rec.get("joint_interfaces", []):
                table.append({
                    "bone": rec["name"],
                    "joint": joint.get("name"),
                    "type": joint.get("type"),
                    "mount_point": joint.get("mount_point"),
                })
        return table

    def _reference_audit(self, records: List[Dict[str, Any]]) -> Dict[str, Any]:
        missing = [rec["name"] for rec in records if not rec.get("references")]
        return {
            "total_bones": len(records),
            "with_references": len(records) - len(missing),
            "missing_references": missing,
        }
