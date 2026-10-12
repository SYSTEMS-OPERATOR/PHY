"""Independent engineering observations must not become qualification claims."""
from copy import deepcopy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import warnings

from skeleton.anatomical_inventory import expected_bones, source_record_paths
from skeleton.base import BoneSpec
from skeleton.bones import load_field
from skeleton.exporters import ExporterAgent
from skeleton.exporters.exporter_agent import json_bytes, sha256
from skeleton.field import SkeletonField
from skeleton.validation.engineering_audit import engineering_audit, write_reports
from skeleton.visualization import bone_equivalence
from skeleton.visualization.studio_model import build_armature


ROOT = Path(__file__).resolve().parents[1]


def fixture(uid="BONE_HYOID", name="Hyoid", **changes):
    values = dict(
        unique_id=uid, name=name, bone_type="test", location={"region": "test"},
        articulations=[], function=[], notable_features=[], developmental_notes="synthetic",
        variations="", dimensions={"length_mm": 100, "width_mm": 10, "thickness_mm": 10},
        geometry={"shape": "box"}, material={"name": "test", "density": 1800},
        physics={"mass_kg": 0.018}, connections={"parent": None, "children": []},
        joint_interfaces=[{"name": "synthetic"}], mount_points=[{"name": "synthetic"}],
        manufacturing_notes=["Software test fixture, not anatomical evidence"], references=["test-evidence"],
    )
    values.update(changes)
    return BoneSpec(**values)


def by_id(report, uid):
    return next(row for row in report["bones"] if row["bone_id"] == uid)


class EngineeringAuditTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            cls.field = load_field()
        cls.model = build_armature()
        cls.report = engineering_audit(cls.field, cls.model["parts"])

    def test_shared_inventory_preserves_studio_contract_and_checked_in_coverage(self):
        self.assertIs(expected_bones, bone_equivalence.expected_bones)
        self.assertIs(source_record_paths, bone_equivalence.source_record_paths)
        expected = json.loads((ROOT / "PROJECTS/PHY_F28/reports/BONE_EQUIVALENCE.json").read_text())
        self.assertEqual(self.model["bone_equivalence"], expected)
        self.assertEqual(len(expected_bones()), 206)
        self.assertEqual(len(source_record_paths()), 193)

    def test_real_checkpoint_reports_independent_counts_and_exact_canonical_fingerprint(self):
        observed = {key: metric["count"] for key, metric in self.report["metrics"].items()}
        self.assertEqual(observed, {
            "individual_display_identity": 206, "source_module_present": 193,
            "canonical_record_present": 193, "declared_dimensions_complete": 36,
            "dataset_binding_present": 21, "reference_declaration_present": 0,
            "dimensional_primitive_available": 30, "declared_anatomical_references_resolved": 24,
            "record_validation_pass": 0,
        })
        self.assertEqual(len(self.report["missing_canonical_records"]), 13)
        self.assertEqual(self.report["missing_canonical_records"], self.report["missing_source_records"])
        for key, metric in self.report["metrics"].items():
            self.assertEqual(metric["total"], 206)
            self.assertTrue(metric["meaning"])
            self.assertEqual(sum(region["metrics"][key]["count"] for region in self.report["regions"]), metric["count"])
            self.assertEqual(sum(region["metrics"][key]["total"] for region in self.report["regions"]), 206)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            paths = ExporterAgent(self.field).export_all(root / "dist", root / "reports", root / "exports")
            manifest = json.loads(Path(paths["manifest"]).read_text())
        self.assertEqual(self.report["inputs"]["canonical_records_sha256"], manifest["source"]["source_records_sha256"])
        self.assertFalse(manifest["validated_ready"])
        self.assertFalse(self.report["record_validation"]["summary"]["pass"])

    def test_missing_and_unknown_measurements_are_not_filled_by_proxies(self):
        missing = by_id(self.report, "BONE_MAXILLA_R")
        self.assertEqual(missing["display"]["representation"], "individual_project_proxy")
        self.assertEqual(missing["canonical_record"]["status"], "missing")
        self.assertIsNone(missing["source"]["path"])
        self.assertIsNone(missing["dimensions"]["values_mm"])
        self.assertIsNone(missing["canonical_geometry"])
        unknown = by_id(self.report, "BONE_C1")
        self.assertEqual(unknown["dimensions"]["status"], "unknown")
        self.assertTrue(unknown["dimensions"]["unknown_keys"])
        self.assertTrue(all(value is None for value in unknown["dimensions"]["values_mm"].values()))
        parts = deepcopy(self.model["parts"])
        next(part for part in parts if part.get("bone_id") == "BONE_C1")["vertices"][0][0] += 100
        altered = engineering_audit(self.field, parts)
        self.assertEqual(self.report["inputs"]["canonical_records_sha256"], altered["inputs"]["canonical_records_sha256"])
        self.assertNotEqual(self.report["inputs"]["display_parts_sha256"], altered["inputs"]["display_parts_sha256"])
        self.assertEqual(unknown["dimensions"], by_id(altered, "BONE_C1")["dimensions"])

    def test_source_and_dataset_declarations_are_traceable_without_qualification(self):
        for row in self.report["bones"]:
            source = row["source"]
            if source["path"]:
                self.assertEqual(source["sha256"], sha256((ROOT / source["path"]).read_bytes()))
        femur = by_id(self.report, "BONE_FEMUR_R")
        self.assertEqual(femur["dimensions"]["values_mm"], {"length_mm": 420.0, "width_mm": 40.0, "thickness_mm": 40.0})
        self.assertEqual(femur["dimensions"]["metric_source_declarations"]["length_cm"], "Femur")
        self.assertEqual(femur["dataset"]["key"], "Femur")
        self.assertEqual(femur["dataset"]["sha256"], sha256(json_bytes(self.field.get("BONE_FEMUR_R").dataset)))
        self.assertEqual(femur["canonical_geometry"]["status"], "dimensional_estimate")
        scapula = by_id(self.report, "BONE_SCAPULA_R")
        self.assertEqual(scapula["dimensions"]["status"], "complete")
        self.assertEqual(scapula["canonical_geometry"]["status"], "unresolved")
        hyoid = by_id(self.report, "BONE_HYOID")
        self.assertEqual(hyoid["anatomical_references"]["status"], "undeclared")
        self.assertIsNone(hyoid["anatomical_references"]["parent"]["bone_id"])
        self.assertIn("qualified_articulation_and_transforms", hyoid["evidence_needed"])

    def test_complete_synthetic_record_does_not_qualify_anatomy_or_physical_release(self):
        report = engineering_audit(SkeletonField([fixture()]), self.model["parts"])
        self.assertEqual(report["metrics"]["declared_dimensions_complete"]["count"], 1)
        self.assertEqual(report["metrics"]["record_validation_pass"]["count"], 1)
        self.assertEqual(report["metrics"]["reference_declaration_present"]["count"], 1)
        self.assertEqual(len(report["missing_canonical_records"]), 205)
        self.assertEqual(report["qualification"]["dimensional_fidelity"], "unverified")
        self.assertEqual(report["qualification"]["anatomical_geometry"], "unqualified")
        self.assertEqual(report["qualification"]["articulation"], "unqualified")
        self.assertFalse(report["qualification"]["fabrication_released"])
        self.assertEqual(by_id(report, "BONE_HYOID")["dataset"]["status"], "not_selected")

    def test_partial_invalid_and_empty_dimensions_remain_separate_from_completeness(self):
        for values, status, known, unknown, invalid in (
            ({"length_mm": 100, "width_mm": None, "thickness_mm": 10}, "partial", ["length_mm", "thickness_mm"], ["width_mm"], []),
            ({"length_mm": -1, "width_mm": 0, "thickness_mm": 10}, "partial", ["thickness_mm"], [], ["length_mm", "width_mm"]),
            ({}, "unknown", [], [], []),
        ):
            with self.subTest(values=values):
                report = engineering_audit(SkeletonField([fixture(dimensions=values)]), [])
                dimensions = by_id(report, "BONE_HYOID")["dimensions"]
                self.assertEqual((dimensions["status"], dimensions["known_keys"], dimensions["unknown_keys"], dimensions["invalid_keys"]),
                                 (status, known, unknown, invalid))
                self.assertEqual(report["metrics"]["declared_dimensions_complete"]["count"], 0)
                self.assertEqual(report["metrics"]["individual_display_identity"]["count"], 0)

    def test_dataset_binding_is_distinct_from_dimensions_and_citations(self):
        bone = fixture(dimensions={"length_mm": None, "width_mm": None, "thickness_mm": None})
        bone.dataset = {"Hyoid": {}}
        bone.dataset_key = "Hyoid"
        report = engineering_audit(SkeletonField([bone]), [])
        self.assertEqual(report["metrics"]["dataset_binding_present"]["count"], 1)
        self.assertEqual(report["metrics"]["declared_dimensions_complete"]["count"], 0)
        bone.dataset_key = "missing"
        report = engineering_audit(SkeletonField([bone]), [])
        self.assertEqual(report["metrics"]["dataset_binding_present"]["count"], 0)
        self.assertEqual(by_id(report, bone.unique_id)["dataset"]["status"], "unbound")

    def test_malformed_reference_source_types_do_not_become_citation_coverage(self):
        for references in ("not-a-citation-list", [""], ["valid", 4], ("tuple-reference",)):
            with self.subTest(references=references):
                report = engineering_audit(SkeletonField([fixture(references=references)]), [])
                row = by_id(report, "BONE_HYOID")
                self.assertEqual(report["metrics"]["reference_declaration_present"]["count"], 0)
                self.assertEqual(row["references"]["status"], "invalid")
                self.assertIn("invalid_field_types", row["record_validation"]["blocker_categories"])

    def test_side_ambiguity_cycles_and_malformed_declarations_do_not_count_as_resolved(self):
        right = fixture("BONE_FEMUR_R", "Femur")
        left = fixture("BONE_FEMUR_L", "Femur")
        child = fixture("BONE_HYOID", connections={"parent": "Femur", "children": []})
        report = engineering_audit(SkeletonField([right, left, child]), [])
        parent = by_id(report, child.unique_id)["anatomical_references"]["parent"]
        self.assertEqual(parent["status"], "ambiguous")
        self.assertEqual(parent["candidates"], ["BONE_FEMUR_L", "BONE_FEMUR_R"])
        self.assertEqual(report["metrics"]["declared_anatomical_references_resolved"]["count"], 0)
        child.connections = {"parent": None, "children": "not-an-array"}
        report = engineering_audit(SkeletonField([child]), [])
        relationship = by_id(report, child.unique_id)["anatomical_references"]
        self.assertEqual(relationship["status"], "unresolved")
        self.assertTrue(relationship["issues"])
        right.connections = {"parent": left.unique_id, "children": []}
        left.connections = {"parent": right.unique_id, "children": []}
        report = engineering_audit(SkeletonField([right, left]), [])
        self.assertTrue(report["identity_validation"]["parent_cycles"])
        self.assertEqual(report["metrics"]["declared_anatomical_references_resolved"]["count"], 0)
        self.assertTrue(all(by_id(report, bone.unique_id)["anatomical_references"]["parent_cycle"] for bone in (right, left)))

    def test_unknown_duplicate_and_nonfinite_input_are_refused(self):
        with self.assertRaisesRegex(ValueError, "Unknown canonical adult"):
            engineering_audit(SkeletonField([fixture("NOT_AN_ADULT_ID")]), [])
        field = SkeletonField([fixture()])
        field.bones["duplicate-domain"] = deepcopy(field.bones["BONE_HYOID"])
        with self.assertRaisesRegex(ValueError, "Duplicate canonical adult"):
            engineering_audit(field, [])
        for value in (float("nan"), float("inf"), True, "100"):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, "Cannot audit canonical record BONE_HYOID"):
                engineering_audit(SkeletonField([fixture(dimensions={"length_mm": value})]), [])
        empty = engineering_audit(SkeletonField(), [])
        self.assertEqual(empty["metrics"]["canonical_record_present"]["count"], 0)
        self.assertFalse(empty["record_validation"]["summary"]["pass"])
        self.assertEqual(len(empty["missing_canonical_records"]), 206)

    def test_audit_is_pure_order_independent_and_runtime_activity_does_not_change_it(self):
        field = deepcopy(self.field)
        before = json_bytes([bone.to_fabrication_record() for bone in field.bones.values()])
        engineering_audit(field, self.model["parts"])
        self.assertEqual(before, json_bytes([bone.to_fabrication_record() for bone in field.bones.values()]))
        femur = field.get("BONE_FEMUR_R")
        femur.update_state(position=(1, 2, 3), load=(5, 6, 7))
        femur.transcend("test-only runtime activity")
        femur.geometry["verts"] = [[1, 2, 3]]
        field.bones = dict(reversed(list(field.bones.items())))
        report = engineering_audit(field, reversed(self.model["parts"]))
        self.assertEqual(json_bytes(report), json_bytes(self.report))

    def test_source_lookup_diagnoses_duplicate_and_unexpected_identity(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bones = root / "skeleton/bones"
            bones.mkdir(parents=True)
            one = bones / "bone_one.py"
            one.write_text("bone = BoneSpec(unique_id='BONE_C1')\n")
            self.assertEqual(source_record_paths(root), {"BONE_C1": "skeleton/bones/bone_one.py"})
            (bones / "bone_two.py").write_text(one.read_text())
            with self.assertRaisesRegex(ValueError, "duplicated source bone ID"):
                engineering_audit(SkeletonField(), [], root=root)
            (bones / "bone_two.py").write_text("bone = BoneSpec(unique_id='BONE_IMAGINARY')\n")
            with self.assertRaisesRegex(ValueError, "unknown source bone IDs"):
                engineering_audit(SkeletonField(), [], root=root)

    def test_report_package_is_repeatable_hash_verified_and_cli_failure_preserves_it(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            first = write_reports(self.report, root / "one")
            second = write_reports(self.report, root / "two")
            for key in first:
                self.assertEqual(Path(first[key]).read_bytes(), Path(second[key]).read_bytes())
            manifest = json.loads(Path(first["manifest"]).read_text())
            self.assertEqual(manifest["inputs"], self.report["inputs"])
            for name, metadata in manifest["files"].items():
                self.assertEqual(metadata["sha256"], sha256((Path(first["manifest"]).parent / name).read_bytes()))
            command = [sys.executable, str(ROOT / "bin/audit_engineering.py"), "--output-root", str(root / "cli")]
            result = subprocess.run(command, cwd=directory, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            response = json.loads(result.stdout)
            self.assertTrue(response["completed"])
            self.assertFalse(response["fabrication_released"])
            previous = {key: Path(path).read_bytes() for key, path in response["paths"].items()}
            result = subprocess.run(command + ["--dataset", "missing-test-dataset"], cwd=directory, capture_output=True, text=True)
            self.assertEqual(result.returncode, 1, result.stderr)
            self.assertFalse(json.loads(result.stdout)["completed"])
            for key, path in response["paths"].items():
                self.assertEqual(previous[key], Path(path).read_bytes())


if __name__ == "__main__":
    unittest.main()
