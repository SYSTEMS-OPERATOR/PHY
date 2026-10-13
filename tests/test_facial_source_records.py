"""Identity coverage may improve without acquiring numerical or physical truth."""
import json
from pathlib import Path
import tempfile
import unittest
import warnings

from skeleton.anatomical_inventory import expected_bones, source_record_paths
from skeleton.base import CANONICAL_UNITS
from skeleton.bones import load_field
from skeleton.exporters import ExportError, ExporterAgent
from skeleton.exporters.exporter_agent import json_bytes, sha256
from skeleton.validation.engineering_audit import engineering_audit
from skeleton.validation.validator_agent import ValidatorAgent
from skeleton.visualization.studio_model import build_armature


ROOT = Path(__file__).resolve().parents[1]
FACIAL_IDS = {f"BONE_{code}_{side}" for code in (
    "MAXILLA", "ZYGOMATIC", "NASAL", "LACRIMAL", "PALATINE", "INFERIOR_NASAL_CONCHA")
    for side in ("L", "R")} | {"BONE_VOMER"}


class FacialSourceRecordTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            cls.field = load_field()
        cls.records = {bone.unique_id: bone.to_fabrication_record() for bone in cls.field.bones.values()}
        cls.model = build_armature()
        cls.provenance = json.loads((ROOT / "references/facial_identity_provenance.json").read_text())

    def test_exact_missing_identity_set_closes_canonical_coverage(self):
        self.assertEqual(len(FACIAL_IDS), 13)
        self.assertEqual(set(self.records), {row["bone_id"] for row in expected_bones()})
        self.assertEqual(set(source_record_paths()), set(self.records))
        self.assertEqual(len(self.records), 206)
        self.assertEqual({row["bone_id"] for row in self.provenance["records"]}, FACIAL_IDS)
        self.assertEqual(self.model["bone_equivalence"]["missing_source_records"], [])

    def test_new_records_declare_identity_evidence_and_keep_physical_fields_unknown(self):
        for uid in sorted(FACIAL_IDS):
            bone = self.field.get(uid)
            record = self.records[uid]
            with self.subTest(uid=uid):
                self.assertEqual(record["units"], CANONICAL_UNITS)
                self.assertEqual(record["latin_name"], "unresolved")
                self.assertEqual(record["bone_type"], "unclassified")
                self.assertEqual(record["dimensions"], {"length_mm": None, "width_mm": None, "thickness_mm": None})
                self.assertIsNone(record["material"]["density"])
                self.assertIsNone(record["physics"]["mass_kg"])
                self.assertIsNone(record["physics"]["inertia_kg_m2"])
                self.assertEqual(record["geometry"], {"status": "unresolved", "reason": "no source geometry"})
                self.assertEqual(record["connections"], {"parent": None, "children": []})
                self.assertEqual(record["joint_interfaces"], [])
                self.assertEqual(record["mount_points"], [])
                self.assertEqual(record["source_ids"], [self.provenance["source_id"]])
                self.assertEqual(record["references"], [self.provenance["source_url"]])
                self.assertIsNone(bone.dataset_key)
                self.assertEqual(bone.metric_sources, {})
                self.assertEqual(record["tolerance"], {"default_mm": 0.5})
                self.assertTrue(any("policy placeholder" in note for note in record["manufacturing_notes"]))
                self.assertEqual(bone.location["side"], "left" if uid.endswith("_L") else "right" if uid.endswith("_R") else "midline")

    def test_evidence_catalog_is_scoped_to_identity_and_matches_local_records(self):
        catalog = self.provenance
        self.assertEqual(catalog["evidence_scope"], ["adult_facial_identity_names", "paired_or_unpaired_identity_accounting"])
        self.assertEqual(catalog["source_url"], "https://openstax.org/books/anatomy-and-physiology-2e/pages/7-2-the-skull")
        self.assertIn("dimensions", catalog["does_not_qualify"])
        self.assertIn("mechanical_relationships", catalog["does_not_qualify"])
        for row in catalog["records"]:
            self.assertEqual(source_record_paths()[row["bone_id"]], row["source_record"])
            self.assertEqual(self.records[row["bone_id"]]["name"], row["name"])

    def test_established_193_source_files_and_records_match_current_evidence_checkpoint(self):
        # These identities predate the facial continuation. PHY-DIM-072 supplies
        # reviewed dimensions for 32 of them; its separate baseline test retains
        # every original complete map and rejects unsupported dimensional changes.
        old = [self.records[uid] for uid in sorted(set(self.records) - FACIAL_IDS)]
        self.assertEqual(len(old), 193)
        self.assertEqual(sha256(json_bytes(old)), "a0799a5e7af2358053efb490bce0ba2d48c74469756a2c41925938948c9244cd")
        paths = source_record_paths()
        old_files = {paths[uid]: sha256((ROOT / paths[uid]).read_bytes()) for uid in sorted(set(paths) - FACIAL_IDS)}
        self.assertEqual(sha256(json_bytes(old_files)), "62e4cdc986401cca00007722f36915fb031286783d0885803bfeedace1034f7e")

    def test_every_previous_display_geometry_and_metadata_except_source_presence_is_preserved(self):
        parts = [{key: value for key, value in part.items() if key not in ("source_record", "source_record_status")}
                 for part in self.model["parts"]]
        self.assertEqual(len(parts), 230)
        self.assertEqual(sha256(json_bytes(parts)), "7cf5db008506451c05b7b5a4070fa384018d66e679c4edd7a26ca70f95f26291")
        for part in self.model["parts"]:
            if part.get("bone_id") in FACIAL_IDS:
                self.assertEqual(part["source_record_status"], "present")
                self.assertEqual(part["dimensional_fidelity"], "unverified")
                self.assertEqual(part["physical_evidence"], "unmeasured")

    def test_only_supported_audit_dimensions_improve(self):
        report = engineering_audit(self.field, self.model["parts"])
        self.assertEqual({key: value["count"] for key, value in report["metrics"].items()}, {
            "individual_display_identity": 206, "source_module_present": 206,
            "canonical_record_present": 206, "declared_dimensions_complete": 40,
            "dataset_binding_present": 21, "reference_declaration_present": 45,
            "dimensional_primitive_available": 30, "declared_anatomical_references_resolved": 24,
            "record_validation_pass": 0,
        })
        self.assertEqual(report["missing_source_records"], [])
        self.assertEqual(report["missing_canonical_records"], [])
        self.assertFalse(report["qualification"]["fabrication_released"])
        self.assertEqual(report["qualification"]["mechanical_transforms"], "unresolved")

    def test_new_unknown_density_is_a_blocker_and_existing_unknowns_remain_visible(self):
        report = ValidatorAgent(self.field).run()
        self.assertEqual(report["summary"]["total_bones"], 206)
        self.assertEqual(report["summary"]["failed_checks"], 9)
        self.assertEqual({row["bone"] for row in report["material_incompleteness"]}, FACIAL_IDS)
        self.assertEqual(len({row["bone"] for row in report["missing_dimension_values"]}), 166)
        self.assertEqual(len(report["missing_mount_points"]), 206)
        self.assertEqual(len(report["missing_joint_interfaces"]), 206)
        self.assertEqual(len(report["missing_references"]), 161)
        self.assertFalse(report["export_readiness"]["ready"])

    def test_review_export_has_206_records_and_null_transforms_while_validated_stays_blocked(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            exporter = ExporterAgent(self.field)
            with self.assertRaises(ExportError) as raised:
                exporter.export_all(root / "dist", root / "reports", root / "exports", mode="validated")
            self.assertEqual(raised.exception.code, "validation_failed")
            self.assertFalse((root / "dist/export_manifest.json").exists())
            paths = exporter.export_all(root / "dist", root / "reports", root / "exports", mode="review")
            manifest = json.loads(Path(paths["manifest"]).read_text())
            self.assertEqual(manifest["record_count"], 206)
            self.assertFalse(manifest["fabrication_released"])
            self.assertFalse(manifest["validated_ready"])
            frames = {row["frame_id"]: row for row in json.loads(Path(paths["tf_tree"]).read_text())}
            for uid in FACIAL_IDS:
                self.assertIsNone(frames[uid]["parent"])
                self.assertIsNone(frames[uid]["origin_mm"])
                self.assertIsNone(frames[uid]["rotation_rpy_rad"])
                self.assertEqual(frames[uid]["transform_status"], "unresolved")

    def test_checked_in_review_package_matches_source_snapshot_and_manifest_hashes(self):
        manifest_path = ROOT / "dist/export_manifest.json"
        manifest = json.loads(manifest_path.read_text())
        self.assertEqual(manifest["mode"], "review")
        self.assertEqual(manifest["record_count"], 206)
        self.assertFalse(manifest["validated_ready"])
        self.assertFalse(manifest["fabrication_released"])
        ordered = [self.records[uid] for uid in sorted(self.records)]
        self.assertEqual(manifest["source"]["source_records_sha256"], sha256(json_bytes(ordered)))
        for entry in manifest["files"].values():
            self.assertEqual(entry["sha256"], sha256((manifest_path.parent / entry["path"]).read_bytes()))


if __name__ == "__main__":
    unittest.main()
