"""Structural references cannot silently acquire pose, fit or fabrication truth."""
from copy import deepcopy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import warnings

from skeleton.anatomical_inventory import source_record_paths
from skeleton.bones import load_field
from skeleton.exporters.exporter_agent import json_bytes, sha256
from skeleton.validation.craniofacial_audit import (
    craniofacial_audit, load_catalog, validate_catalog, write_reports,
)
from skeleton.visualization.studio_model import build_armature

ROOT = Path(__file__).resolve().parents[1]


def records(field):
    return sorted((bone.to_fabrication_record() for bone in field.bones.values()),
                  key=lambda row: row["unique_id"])


class CraniofacialAuditTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            cls.field = load_field()
        cls.model = build_armature(height_mm=1676.4, span_equals_height=True)
        cls.catalog = load_catalog()
        cls.report = craniofacial_audit(cls.field, cls.model)

    def test_orbits_name_all_seven_bones_and_preserve_subject_sides(self):
        regions = {row["region_id"]: row for row in self.catalog["regions"]}
        common = {"BONE_FRONTAL", "BONE_ETHMOID", "BONE_SPHENOID"}
        for side in ("r", "l"):
            orbit = regions["orbit_" + side]
            self.assertEqual({c["bone_id"] for c in orbit["contributions"]}, common | {
                "BONE_" + code + "_" + side.upper()
                for code in ("ZYGOMATIC", "MAXILLA", "PALATINE", "LACRIMAL")})
            self.assertIsNone(orbit["spatial_definition"])
        bridge = {c["bone_id"] for c in regions["nasal_bridge"]["contributions"]}
        self.assertEqual(bridge, {"BONE_NASAL_R", "BONE_NASAL_L"})
        self.assertEqual({c["bone_id"] for c in regions["nasal_septum"]["contributions"]},
                         {"BONE_ETHMOID", "BONE_VOMER"})
        self.assertEqual({c["bone_id"] for c in regions["hard_palate"]["contributions"]},
                         {"BONE_MAXILLA_R", "BONE_MAXILLA_L", "BONE_PALATINE_R", "BONE_PALATINE_L"})

    def test_tmjs_share_one_adult_mandible_and_retain_rotation_translation_and_disc(self):
        joints = self.report["articulations"]
        self.assertEqual(set.intersection(*(set(j["bone_ids"]) for j in joints)), {"BONE_MANDIBLE"})
        self.assertEqual(self.report["mandibular_system"]["shared_moving_bone_id"], "BONE_MANDIBLE")
        for joint in joints:
            self.assertEqual(joint["anatomical_type"], "synovial")
            self.assertEqual(set(joint["motion_components"]), {"rotation", "translation"})
            self.assertEqual(joint["non_bone_components"], ["articular_disc"])
            self.assertIsNone(joint["engineering_definition"])
        for change in ({"anatomical_type": "fixed"}, {"motion_components": ["rotation"]},
                       {"bone_ids": ["BONE_MANDIBLE", "BONE_TEMP_L"]},
                       {"non_bone_components": []}, {"side": "left"}):
            altered = deepcopy(self.catalog)
            altered["articulations"][0].update(change)
            with self.subTest(change=change), self.assertRaises(ValueError):
                validate_catalog(altered)

    def test_perception_evidence_cannot_qualify_anatomy_or_joint_motion(self):
        for key in ("regions", "articulations"):
            altered = deepcopy(self.catalog)
            altered[key][0]["source_id"] = "PALLETT_LINK_LEE_2010_PERCEPTION"
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, "required scope"):
                validate_catalog(altered)
        altered = deepcopy(self.catalog)
        altered["sources"][2]["supports"] = ["structural_participation"]
        with self.assertRaisesRegex(ValueError, "evidence kind or scope"):
            validate_catalog(altered)

    def test_unknown_ids_wrong_sides_and_unsupported_numeric_definitions_fail(self):
        mutations = [
            lambda c: c["regions"][0]["contributions"][0].update(bone_id="BONE_UNKNOWN"),
            lambda c: c["regions"][0]["contributions"][0].update(bone_id="BONE_FEMUR_R"),
            lambda c: c["regions"][0]["contributions"][1].update(bone_id="BONE_ZYGOMATIC_L"),
            lambda c: c["regions"][0].update(spatial_definition={"origin_mm": [0, 0, 0]}),
            lambda c: c["articulations"][0].update(engineering_definition={"axis": [1, 0, 0]}),
            lambda c: c["regions"][0].update(origin_mm=[0, 0, 0]),
            lambda c: c["regions"][0].update(source_id="UNKNOWN_REFERENCE"),
            lambda c: c["regions"][1].update(region_id="orbit_r"),
            lambda c: c["articulations"][1].update(articulation_id="tmj_r"),
        ]
        for index, mutate in enumerate(mutations):
            altered = deepcopy(self.catalog)
            mutate(altered)
            with self.subTest(index=index), self.assertRaises(ValueError):
                validate_catalog(altered)

    def test_report_separates_structural_coverage_from_unqualified_physics_and_fit(self):
        self.assertEqual(self.report["counts"], {
            "structural_regions": 7, "qualitative_articulations": 2,
            "participating_bone_identities": 17, "canonical_records_present": 17,
            "individual_display_proxies": 17, "spatially_defined_regions": 0,
            "mechanically_defined_articulations": 0})
        q = self.report["qualification"]
        self.assertFalse(q["fabrication_released"])
        self.assertFalse(q["facial_ratios_adopted"])
        self.assertEqual(q["mechanical_transforms"], "unresolved")
        self.assertEqual(q["external_feature_fit"], "unresolved")
        self.assertEqual(q["canonical_face_landmarks"], "unbound")
        rows = {r["bone_id"]: r for r in self.report["bones"]}
        self.assertEqual(rows["BONE_MAXILLA_R"]["dimensions"]["status"], "unknown")
        self.assertEqual(rows["BONE_MANDIBLE"]["canonical_geometry"]["status"], "dimensional_estimate")
        # Regional anatomy must not repair a legacy ambiguous parent reference.
        parent = rows["BONE_MANDIBLE"]["anatomical_references"]["parent"]
        self.assertEqual(parent["reference"], "temporal")
        self.assertEqual(parent["status"], "missing")
        self.assertIsNone(parent["bone_id"])

    def test_head_guide_keeps_measured_external_means_and_project_height_distinct(self):
        guide = self.report["head_guide_context"]
        self.assertEqual((guide["height_mm"], guide["cohort_n"], guide["age_years"]), (1676.4, 92, 28))
        self.assertIn("Not a representative", guide["population"])
        self.assertEqual(guide["canon_effect"], "none")
        axes = {a["axis"]: a for a in guide["axes"]}
        envelope = next(p for p in self.model["parts"] if p["id"] == "head_envelope")
        for axis, index in (("x", 0), ("y", 1), ("z", 2)):
            width = envelope["bounds_mm"]["max"][index] - envelope["bounds_mm"]["min"][index]
            self.assertAlmostEqual(axes[axis]["guide_value_mm"], width, places=4)
        self.assertEqual(axes["x"]["basis"], "external_cohort_marginal_mean")
        self.assertAlmostEqual(axes["x"]["statistics"]["mean_mm"], 147.945652)
        self.assertEqual(axes["z"]["basis"], "project_design_choice")
        self.assertEqual(axes["z"]["base_value_mm"], 220)
        self.assertNotIn("statistics", axes["z"])
        for change in ("reference", "scale"):
            altered = deepcopy(self.model)
            if change == "reference":
                altered["reference"]["statistics"]["headbreadth"]["mean_mm"] += 1
            else:
                altered["scale_from_mean"] *= 1.01
            with self.subTest(change=change), self.assertRaises(ValueError):
                craniofacial_audit(self.field, altered)

    def test_audit_preserves_all_canonical_records_source_files_catalog_and_display_parts(self):
        record_hash = sha256(json_bytes(records(self.field)))
        model_hash = sha256(json_bytes(self.model))
        catalog_hash = sha256(json_bytes(self.catalog))
        files = {path: sha256((ROOT / path).read_bytes()) for path in source_record_paths().values()}
        report = craniofacial_audit(self.field, self.model, catalog=self.catalog)
        self.assertEqual(sha256(json_bytes(records(self.field))), record_hash)
        self.assertEqual(sha256(json_bytes(self.model)), model_hash)
        self.assertEqual(sha256(json_bytes(self.catalog)), catalog_hash)
        self.assertEqual(report["inputs"]["catalog_sha256"], catalog_hash)
        self.assertEqual(report["inputs"]["canonical_records_sha256"], record_hash)
        self.assertEqual(files, {p: sha256((ROOT / p).read_bytes()) for p in files})
        manifest = json.loads((ROOT / "dist/export_manifest.json").read_text())
        self.assertEqual(record_hash, manifest["source"]["source_records_sha256"])

    def test_missing_records_and_proxies_are_reported_without_inventing_replacements(self):
        field, model = deepcopy(self.field), deepcopy(self.model)
        key = next(k for k, bone in field.bones.items() if bone.unique_id == "BONE_MAXILLA_R")
        del field.bones[key]
        model["parts"] = [p for p in model["parts"] if p.get("bone_id") != "BONE_TEMP_L"]
        report = craniofacial_audit(field, model)
        self.assertEqual(report["counts"]["canonical_records_present"], 16)
        self.assertEqual(report["counts"]["individual_display_proxies"], 16)
        regions = {r["region_id"]: r for r in report["regions"]}
        self.assertEqual(regions["orbit_r"]["missing_canonical_records"], ["BONE_MAXILLA_R"])
        self.assertEqual(regions["zygomatic_arches"]["missing_display_proxies"], ["BONE_TEMP_L"])
        joints = {j["articulation_id"]: j for j in report["articulations"]}
        self.assertEqual(joints["tmj_l"]["missing_display_proxies"], ["BONE_TEMP_L"])
        self.assertIsNone(joints["tmj_l"]["engineering_definition"])

    def test_deterministic_manifested_publication_and_rejection_preserve_previous_review(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            outputs = []
            for name in ("one", "two"):
                paths = write_reports(self.report, root / name)
                outputs.append({key: Path(path).read_bytes() for key, path in paths.items()})
                manifest = json.loads(Path(paths["manifest"]).read_text())
                self.assertFalse(manifest["fabrication_released"])
                for filename, entry in manifest["files"].items():
                    self.assertEqual(entry["sha256"], sha256((Path(paths["json"]).parent / filename).read_bytes()))
            self.assertEqual(outputs[0], outputs[1])
            altered = deepcopy(self.report)
            altered["qualification"]["fabrication_released"] = True
            with self.assertRaisesRegex(ValueError, "unqualified"):
                write_reports(altered, root / "one")
            for key, path in paths.items():
                self.assertEqual(Path(path).read_bytes(), outputs[1][key])
            for path in (root / "one/reports").iterdir():
                key = "manifest" if "manifest" in path.name else "markdown" if path.suffix == ".md" else "json"
                self.assertEqual(path.read_bytes(), outputs[0][key])

    def test_cli_runs_from_outside_checkout_and_invalid_height_preserves_package(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            command = [sys.executable, str(ROOT / "bin/audit_craniofacial.py"), "--output-root", str(root / "output")]
            run = subprocess.run(command, cwd=root, capture_output=True, text=True)
            self.assertEqual(run.returncode, 0, run.stderr)
            status = json.loads(run.stdout)
            self.assertEqual(status["counts"]["participating_bone_identities"], 17)
            before = {key: Path(path).read_bytes() for key, path in status["paths"].items()}
            rejected = subprocess.run(command + ["--height-mm", "nan"], cwd=root, capture_output=True, text=True)
            self.assertEqual(rejected.returncode, 1)
            self.assertFalse(json.loads(rejected.stdout)["completed"])
            self.assertEqual(before, {key: Path(path).read_bytes() for key, path in status["paths"].items()})

    def test_tracked_review_matches_current_inputs_and_its_manifest(self):
        path = ROOT / "reports/craniofacial_audit.json"
        self.assertEqual(path.read_bytes(), json_bytes(self.report))
        manifest = json.loads((path.parent / "craniofacial_audit_manifest.json").read_text())
        self.assertEqual(manifest["inputs"], self.report["inputs"])
        for filename, entry in manifest["files"].items():
            self.assertEqual(entry["sha256"], sha256((path.parent / filename).read_bytes()))


if __name__ == "__main__":
    unittest.main()
