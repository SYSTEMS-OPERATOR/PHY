"""Spine feature evidence, dimensional semantics, topology and offline reports."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import warnings

from geometry.geometry_agent import GeometryAgent
from skeleton.bones import load_field
from skeleton.exporters.exporter_agent import json_bytes
from skeleton.field import SkeletonField
from skeleton.validation.spine_audit import (
    ROOT, FEATURES_PATH, SOURCE_REVIEW_PATH, SPINE_ORDER, audit_spine, load_spine_features,
)
from skeleton.validation.dimension_evidence import load_evidence, read_json


class SpineAuditTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            cls.field = load_field()
        cls.catalog, cls.features = load_spine_features()
        cls.report = audit_spine(cls.field)

    def test_every_adult_spine_identity_has_cited_features_without_geometric_adoption(self):
        report = self.report
        self.assertEqual(report["counts"], {"expected_spine_bones": 26, "present_spine_bones": 26,
                                          "admitted_feature_fields_checked": 102,
                                          "published_rows_reviewed": 105, "consistency_issues": 0})
        self.assertTrue(report["consistency_pass"])
        self.assertEqual([row["bone_id"] for row in report["bones"]], list(SPINE_ORDER))
        self.assertEqual(len({row["bone_id"] for row in report["bones"]}), 26)
        self.assertFalse(report["all_dimensions_verified"])
        self.assertFalse(report["qualification"]["fabrication_released"])
        self.assertEqual(report["qualification"]["physical_evidence"], "unmeasured")
        self.assertTrue(all(row["feature_geometry"] is None and row["mechanical_definition"] is None
                            and row["notable_features"] for row in report["bones"]))

    def test_atlas_axis_transitional_facets_and_coccygeal_variability_remain_distinct(self):
        f = self.features
        self.assertIn("no vertebral body or spinous process", " ".join(f["BONE_C1"]["features"]))
        self.assertIn("Dens (odontoid process)", " ".join(f["BONE_C2"]["features"]))
        self.assertIn("rib 1", " ".join(f["BONE_T1"]["features"]))
        self.assertIn("may be absent", " ".join(f["BONE_T9"]["features"]))
        self.assertIn("varies", " ".join(f["BONE_T10"]["features"]))
        self.assertIn("No transverse costal facets", " ".join(f["BONE_T11"]["features"]))
        self.assertIn("no transverse costal facets", " ".join(f["BONE_T12"]["features"]))
        self.assertIn("fusion is variable", " ".join(f["BONE_COCCYX"]["features"]))
        self.assertIn("straight", " ".join(f["BONE_COCCYX"]["features"]).lower())

    def test_anatomical_superior_references_resolve_without_creating_signal_links(self):
        rows = self.report["bones"]
        self.assertEqual(rows[0]["anatomical_parent"]["bone_id"], "BONE_OCCIPITAL")
        self.assertEqual(rows[-1]["anatomical_parent"]["bone_id"], "BONE_SACRUM")
        for row in rows:
            self.assertEqual(row["anatomical_parent"]["bone_id"], row["expected_superior_reference"])
            self.assertEqual(self.field.get(row["bone_id"]).articulations, [])
            self.assertEqual(self.field.get(row["bone_id"]).entanglement_links, [])

    def test_all_dimensions_preserve_the_admitted_values_and_the_pinned_unknowns(self):
        _, baseline, derived = load_evidence()
        for before in baseline["bones"]:
            uid = before["bone_id"]
            expected = dict(before["dimensions"]["values_mm"])
            expected.update(derived.get(uid, {}))
            self.assertEqual(self.field.get(uid).to_fabrication_record()["dimensions"], expected, uid)
        rows = {row["bone_id"]: row for row in self.report["bones"]}
        self.assertEqual(rows["BONE_SACRUM"]["dimensions_mm"], {"length_mm": 100, "width_mm": 110, "thickness_mm": 10})
        self.assertEqual(rows["BONE_SACRUM"]["dimensional_status"], "legacy_dataset_unverified")
        self.assertEqual(rows["BONE_SACRUM"]["physics"]["mass_provenance"]["status"], "estimated")
        for uid, forbidden in (("BONE_C1", "body_height_mm"), ("BONE_C2", "body_height_mm"),
                               ("BONE_T6", "body_upper_width_mm"), ("BONE_T11", "body_posterior_height_mm")):
            self.assertNotIn(forbidden, rows[uid]["dimensions_mm"])
        rejected = {key for row in self.report["published_source_review"]["numerical_reviews"]
                    for key in row["quarantined_parameters"]}
        self.assertEqual(rejected, {"C2HT", "T6_VBWs", "T11_VBHp"})

    def test_mutated_features_provenance_dimensions_and_parents_are_diagnosed(self):
        for change, expected in (("features", "features"), ("dimension", "dimension_semantics"),
                                 ("parent", "anatomical_parent"), ("cycle", "parent_cycle"),
                                 ("source", "feature_references"), ("provenance", "measurement_provenance")):
            field = deepcopy(self.field)
            atlas = field.get("BONE_C1")
            if change == "features": atlas.notable_features = ["Generic vertebral body"]
            elif change == "dimension": atlas.dimensions["width_mm"] = atlas.dimensions["spinal_canal_width_mm"]
            elif change == "parent": atlas.connections["parent"] = "skull"
            elif change == "cycle": atlas.connections["parent"] = "BONE_C2"
            elif change == "source": atlas.references = []
            else: atlas.metric_sources["spinal_canal_width_mm"] = "unsupported"
            with self.subTest(change=change):
                report = audit_spine(field)
                self.assertFalse(report["consistency_pass"])
                self.assertIn(expected, [issue["check"] for issue in report["issues"]])
                self.assertFalse(report["all_dimensions_verified"])

    def test_partial_inputs_do_not_acquire_missing_records_from_source_files(self):
        field = SkeletonField([deepcopy(self.field.get("BONE_C1"))])
        report = audit_spine(field)
        self.assertEqual(report["counts"]["present_spine_bones"], 1)
        self.assertEqual(len(report["missing_record_ids"]), 25)
        self.assertFalse(report["complete_spine_snapshot"])
        self.assertFalse(report["consistency_pass"])
        self.assertIsNone(report["bones"][0]["anatomical_parent"]["bone_id"])
        field.bones["duplicate"] = deepcopy(field.get("BONE_C1"))
        with self.assertRaisesRegex(ValueError, "duplicate"):
            audit_spine(field)

    def test_catalog_cannot_qualify_features_or_reuse_stale_native_snapshot_hashes(self):
        original = read_json
        for kind in ("geometry", "hash"):
            def changed(path):
                data = original(path)
                if kind == "geometry" and Path(path) == ROOT / FEATURES_PATH:
                    data["feature_sets"][0]["feature_geometry"] = {"radius_mm": 20}
                if kind == "hash" and Path(path) == ROOT / SOURCE_REVIEW_PATH:
                    data["numerical_reviews"][0]["native_snapshot"]["sha256"] = "0" * 64
                return data
            with self.subTest(kind=kind), patch("skeleton.validation.spine_audit.read_json", side_effect=changed), self.assertRaises(ValueError):
                audit_spine(self.field)

    def test_audit_is_pure_and_record_order_independent(self):
        before = deepcopy(self.field.__dict__)
        reordered = deepcopy(self.field)
        reordered.bones = dict(reversed(list(reordered.bones.items())))
        self.assertEqual(json_bytes(self.report), json_bytes(audit_spine(reordered)))
        self.assertEqual(self.field.__dict__, before)
        GeometryAgent(reordered.get("BONE_SACRUM")).recompute()
        reordered.broadcast(3)
        self.assertEqual(json_bytes(self.report), json_bytes(audit_spine(reordered)))

    def test_offline_cli_determinism_hashes_and_failed_input_preserve_previous_reports(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            command = [sys.executable, str(ROOT / "bin/audit_spine.py")]
            results = []
            for suffix in ("one", "two"):
                run = subprocess.run(command + ["--output-root", str(root / suffix), "--require-consistent"],
                                     cwd=directory, capture_output=True, text=True)
                self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
                self.assertFalse(json.loads(run.stdout)["all_dimensions_verified"])
                results.append({p.name: p.read_bytes() for p in (root / suffix / "reports").iterdir()})
            self.assertEqual(*results)
            manifest = json.loads(results[0]["spine_audit_manifest.json"])
            for name, meta in manifest["files"].items():
                self.assertEqual(meta["sha256"], hashlib.sha256(results[0][name]).hexdigest())
                self.assertEqual(meta["size_bytes"], len(results[0][name]))
            failed = subprocess.run(command + ["--output-root", str(root / "one"), "--dataset", "missing-spine-fixture"],
                                    cwd=directory, capture_output=True, text=True)
            self.assertEqual(failed.returncode, 1)
            self.assertFalse(json.loads(failed.stdout)["completed"])
            self.assertEqual(results[0], {p.name: p.read_bytes() for p in (root / "one/reports").iterdir()})


if __name__ == "__main__":
    unittest.main()
