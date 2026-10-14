"""Custody, semantic separation and stale-evidence gates for the offline viewer."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
import warnings

from skeleton.bones import load_field
from skeleton.exporters.exporter_agent import json_bytes
from skeleton.visualization.studio_evidence import (
    BASELINE_PATH, build_studio_evidence, studio_build_stamp, validate_nasa,
)
from skeleton.visualization.studio_model import build_armature

ROOT = Path(__file__).resolve().parents[1]


class StudioEvidenceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.parts = build_armature()["parts"]
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            cls.field = load_field()
        cls.evidence = build_studio_evidence(cls.parts, field=cls.field)

    def test_all_identities_and_independent_counts(self):
        e = self.evidence
        self.assertEqual(len(e["bones"]), 206)
        self.assertEqual(e["metrics"]["source_module_present"]["count"], 206)
        self.assertEqual(e["dimension_expansion"]["current_complete"], 40)
        self.assertEqual(len(e["dimension_expansion"]["improved_partial_ids"]), 28)
        self.assertEqual(sum(len(b["measurements"]) for b in e["bones"].values()), 128)
        self.assertEqual(e["metrics"]["record_validation_pass"]["count"], 0)
        self.assertFalse(e["fabrication_released"])
        self.assertEqual(e["canonical_reference"], {
            "version": "1.0.0", "height_mm": 1676.4, "span_mm": 1676.4, "registered": False})

    def test_landmarks_and_native_statistics_are_traced_without_mesh_inference(self):
        e = self.evidence
        for bone in e["bones"].values():
            for field, trace in bone["measurements"].items():
                with self.subTest(bone=bone["bone_id"], field=field):
                    self.assertEqual(bone["dimensions"]["values_mm"][field], trace["native_measurement"]["mean"])
                    self.assertIsNone(trace["derivation"]["canonical_registration"])
                    self.assertEqual(trace["derivation"]["unit_scale"], 1)
                    self.assertTrue(trace["landmark_definition"])
                    self.assertTrue(trace["uncertainty"])
                    self.assertFalse(trace["geometry_qualified"])
                    self.assertFalse(trace["fabrication_qualified"])
        # The inconsistent female T6 upper-width row was withheld, not repaired.
        t6 = e["bones"]["BONE_T6"]
        self.assertNotIn("body_upper_width_mm", t6["measurements"])
        self.assertIsNone(t6["dimensions"]["values_mm"]["width_mm"])
        self.assertEqual(t6["dimensions"]["status"], "partial")
        self.assertEqual(e["bones"]["BONE_MAXILLA_R"]["dimensions"]["status"], "unknown")

    def test_legacy_complete_values_are_not_reclassified_as_measurement_protocols(self):
        b = self.evidence["bones"]["BONE_FEMUR_R"]
        self.assertEqual(b["dimensions"]["status"], "complete")
        self.assertEqual(b["measurements"], {})
        self.assertEqual(b["record_validation"]["status"], "blocked")

    def test_nasa_printed_conditions_and_conflicts_are_retained(self):
        e = self.evidence
        self.assertEqual(e["nasa_counts"], {
            "conditions": {"unsuited": 20, "suited_unpressurized": 16, "suited_pressurized": 16},
            "source_issues": 17, "anthropometry": 11})
        row = next(r for r in e["nasa"]["ranges"] if r["id"] == "unsuited_neck_1979_flexion_extension")
        self.assertEqual(row["endpoints_as_printed_deg"], {"Ex": 65, "Flex": -34})
        self.assertIn("Requires source clarification", row["source_issue"])
        self.assertIsNone(row["mechanical_limit_deg"])

    def test_nasa_rejects_mechanical_adoption_bad_units_and_nonfinite_values(self):
        for key, value in (("mechanical_limit_deg", [-34,65]), ("condition", "unknown"),
                           ("units", "rad"), ("endpoints_as_printed_deg", {"a": True, "b": 25}),
                           ("endpoints_as_printed_deg", {"a": float("nan"), "b": 25})):
            with self.subTest(key=key):
                n = deepcopy(self.evidence["nasa"])
                n["ranges"][0][key] = value
                with self.assertRaises(ValueError): validate_nasa(n)
        n = deepcopy(self.evidence["nasa"])
        n["anthropometry"][0]["values_mm"][0] += 1
        with self.assertRaises(ValueError): validate_nasa(n)

    def test_changed_canonical_snapshot_requires_explicit_baseline_review(self):
        field = deepcopy(self.field)
        bone = next(b for b in field.bones.values() if b.unique_id == "BONE_TALUS_R")
        bone.dimensions["length_mm"] += 1
        with self.assertRaisesRegex(ValueError, "Stale Studio"):
            build_studio_evidence(self.parts, field=field)

    def test_changed_proxy_snapshot_cannot_reuse_pinned_evidence(self):
        parts = deepcopy(self.parts)
        parts[0]["vertices"][0][0] += 1
        with self.assertRaisesRegex(ValueError, "Stale Studio"):
            build_studio_evidence(parts, field=self.field)

    def test_changed_nasa_bytes_fail_custody_before_publication(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            for path in (BASELINE_PATH, Path("references/dim072/catalog.json"),
                         Path("references/nasa_positioning_ranges.json")):
                (root / path).parent.mkdir(parents=True, exist_ok=True)
                (root / path).write_bytes((ROOT / path).read_bytes())
            path = root / "references/nasa_positioning_ranges.json"
            path.write_bytes(path.read_bytes() + b" ")
            with self.assertRaisesRegex(ValueError, "Evidence hash mismatch"):
                build_studio_evidence(self.parts, root=root, field=self.field)

    def test_payload_and_build_fingerprints_are_deterministic(self):
        other = build_studio_evidence(self.parts, field=self.field)
        self.assertEqual(json_bytes(other), json_bytes(self.evidence))
        stamp = studio_build_stamp(other)
        self.assertEqual(stamp, studio_build_stamp(self.evidence))
        self.assertEqual(stamp["evidence_input_sha256"], hashlib.sha256(json_bytes(other["inputs"])).hexdigest())
        self.assertEqual(stamp["inputs"]["studio/src/evidence.js"], hashlib.sha256((ROOT / "studio/src/evidence.js").read_bytes()).hexdigest())


if __name__ == "__main__":
    unittest.main()
