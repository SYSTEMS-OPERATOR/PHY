"""Evidence integrity, semantic limits and preservation of physical unknowns."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import warnings

from skeleton.bones import load_field
from skeleton.exporters.exporter_agent import json_bytes
from skeleton.validation.dimension_evidence import (
    ROOT, CATALOG_PATH, derive_value, expansion_report, load_evidence, read_json,
)
from skeleton.validation.engineering_audit import engineering_audit
from skeleton.visualization.studio_model import build_armature


class DimensionEvidenceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.catalog, cls.baseline, cls.values = load_evidence()
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            cls.field = load_field()
        cls.engineering = engineering_audit(cls.field, build_armature()["parts"])
        cls.report = expansion_report(cls.field, cls.engineering)

    def test_current_inventory_counts_and_every_legacy_map_remain_exact(self):
        report = self.report
        self.assertEqual(report["inventory_count"], 206)
        self.assertEqual(report["baseline_complete"], 36)
        self.assertEqual(report["current_complete"], 40)
        self.assertEqual(report["remaining_to_target"], 32)
        self.assertEqual(len(report["new_complete_ids"]), 4)
        self.assertEqual(len(report["improved_partial_ids"]), 28)
        self.assertEqual(report["new_source_backed_field_count"], 93)
        self.assertEqual(set(report["new_complete_ids"]), {
            "BONE_TALUS_L", "BONE_TALUS_R", "BONE_PATELLA_L", "BONE_PATELLA_R"})
        before = {b["bone_id"]: b for b in self.baseline["bones"]}
        for row in report["rows"]:
            if before[row["bone_id"]]["dimensions"]["status"] == "complete":
                self.assertEqual(row["before_mm"], row["after_mm"])
                self.assertFalse(row["source_backed_complete"])
        self.assertFalse(report["target_met"])
        self.assertEqual(report["record_validation_pass"], 0)
        self.assertFalse(report["fabrication_released"])
        self.assertTrue(all(not row["geometry_qualified"] and not row["fabrication_qualified"]
                            for row in report["rows"]))

    def test_named_partial_spans_do_not_replace_whole_bone_unknowns(self):
        for region in ("L", "T", "C"):
            ids = [uid for uid in self.values if uid.startswith("BONE_"+region)
                   and uid[6:].isdigit()]
            for uid in ids:
                bone = self.field.get(uid)
                dims = bone.to_fabrication_record()["dimensions"]
                self.assertTrue(all(dims[k] is None for k in ("length_mm", "width_mm", "thickness_mm")))
        for tag in ("L", "R"):
            dims = self.field.get("BONE_CALCANEUS_"+tag).to_fabrication_record()["dimensions"]
            self.assertEqual(dims["length_mm"], 66.2)
            self.assertIsNone(dims["width_mm"])
            self.assertIsNone(dims["thickness_mm"])
            rib = self.field.get("BONE_RIB1_"+tag).to_fabrication_record()["dimensions"]
            self.assertIsNone(rib["length_mm"])
            self.assertNotEqual(rib["tubercle_ventral_chord_mm"], rib["tubercle_ventral_arc_mm"])
        self.assertNotIn("body_height_mm", self.values["BONE_C1"])
        self.assertNotIn("body_height_mm", self.values["BONE_C2"])
        self.assertNotIn("body_posterior_height_mm", self.values["BONE_T11"])

    def test_real_side_specific_data_are_preserved_and_pooled_assumptions_are_explicit(self):
        self.assertEqual(self.values["BONE_TALUS_L"]["length_mm"], 49.15)
        self.assertEqual(self.values["BONE_TALUS_R"]["length_mm"], 48.57)
        self.assertEqual(self.values["BONE_PATELLA_L"]["thickness_mm"], 22.89)
        self.assertEqual(self.values["BONE_PATELLA_R"]["thickness_mm"], 22.56)
        candidates = {c["bone_id"]: c for c in self.catalog["candidates"]}
        m = candidates["BONE_PATELLA_L"]["fields"]["length_mm"]
        self.assertEqual(m["side_application"], "pooled_mean_default")
        self.assertTrue(m["side_assumption"])
        self.assertEqual(m["unit_scale"], 1.0)
        self.assertIsNone(m["canonical_registration"])

    def test_source_scope_cohort_revision_rights_and_hashes_are_required(self):
        for key in ("cohort", "revision", "rights_review", "coordinate_frame", "tool_version"):
            bad = deepcopy(self.catalog)
            bad["sources"]["talus_ct_2016"].pop(key)
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, "Missing provenance"):
                load_evidence(catalog=bad)
        bad = deepcopy(self.catalog)
        bad["sources"]["talus_ct_2016"]["kind"] = "external_anthropometry"
        with self.assertRaisesRegex(ValueError, "scope"):
            load_evidence(catalog=bad)
        bad = deepcopy(self.catalog)
        bad["sources"]["talus_ct_2016"]["native_snapshot"]["sha256"] = "0"*64
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            load_evidence(catalog=bad)
        bad = deepcopy(self.catalog)
        bad["sources"]["talus_ct_2016"]["geometry_qualified"] = True
        with self.assertRaisesRegex(ValueError, "does not qualify"):
            load_evidence(catalog=bad)

    def test_semantic_relabeling_and_unreviewed_transforms_are_refused(self):
        bad = deepcopy(self.catalog)
        c = next(c for c in bad["candidates"] if c["bone_id"] == "BONE_C1")
        c["fields"]["width_mm"] = c["fields"].pop("spinal_canal_width_mm")
        with self.assertRaisesRegex(ValueError, "semantics"):
            load_evidence(catalog=bad)
        for key,value in (("unit_scale", 1.026), ("unit_scale", -1),
                          ("unit_scale", True), ("canonical_registration", "SOPHY")):
            bad = deepcopy(self.catalog)
            c = next(c for c in bad["candidates"] if c["bone_id"] == "BONE_TALUS_L")
            c["fields"]["length_mm"][key] = value
            with self.subTest(key=key,value=value), self.assertRaises(ValueError):
                load_evidence(catalog=bad)

    def test_derivation_rejects_invalid_native_values_unknown_units_and_side_mismatch(self):
        native = dict(mean=10.0, sd=1.0, native_units="mm", admissible=True,
                      bone_id="BONE_TALUS_L", side="left")
        mapping = dict(method="published_mean_identity", unit_scale=1.0,
                       canonical_registration=None, meaning="Synthetic software span")
        self.assertEqual(derive_value(native,mapping,"BONE_TALUS_L","left"),10.0)
        for key,value in (("mean",True),("mean","10"),("mean",0),("mean",-1),
                          ("mean",float("nan")),("sd",-1),("native_units",None),
                          ("native_units","cm"),("admissible",False),("side","right")):
            bad = dict(native); bad[key] = value
            with self.subTest(key=key,value=value), self.assertRaises(ValueError):
                derive_value(bad,mapping,"BONE_TALUS_L","left")
        pooled = dict(native,side="pooled",bone_id="BONE_TALUS")
        with self.assertRaisesRegex(ValueError, "bilateral default"):
            derive_value(pooled,mapping,"BONE_TALUS_L","left")

    def test_absent_provenance_regression_and_dropped_unknowns_are_refused(self):
        for kind in ("references", "metric_sources", "regression", "unknown", "unknown_identity", "cycle"):
            field = deepcopy(self.field)
            bone = field.get("BONE_TALUS_L")
            if kind == "references": bone.references = []
            elif kind == "metric_sources": bone.metric_sources = {}
            elif kind == "regression": field.get("BONE_FEMUR_L").dimensions["length_cm"] = None
            elif kind == "unknown": del field.get("BONE_L5").dimensions["width_mm"]
            elif kind == "unknown_identity": bone.unique_id = "BONE_INVENTED"
            else:
                bone.connections["parent"] = "BONE_TALUS_R"
                field.get("BONE_TALUS_R").connections["parent"] = "BONE_TALUS_L"
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                expansion_report(field,self.engineering)

    def test_fact_parser_rejects_duplicate_keys_nonfinite_tokens_and_path_escape(self):
        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory)/"bad.json"
            for value in ('{"a":1,"a":2}', '{"a":NaN}', '{"a":Infinity}'):
                p.write_text(value)
                with self.subTest(value=value), self.assertRaises(ValueError):read_json(p)
        bad = deepcopy(self.catalog)
        bad["sources"]["talus_ct_2016"]["native_snapshot"]["path"] = "../../escape.json"
        with self.assertRaisesRegex(ValueError,"escapes repository"):
            load_evidence(catalog=bad)

    def test_report_repeatability_cli_shortfall_and_manifest_hashes(self):
        self.assertEqual(json_bytes(self.report),json_bytes(expansion_report(self.field,self.engineering)))
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            command = [sys.executable,str(ROOT/"bin/audit_dimension_expansion.py")]
            a = subprocess.run(command+["--output-root",str(base/"one")],cwd=directory,text=True,capture_output=True)
            b = subprocess.run(command+["--output-root",str(base/"two"),"--require-target"],cwd=directory,text=True,capture_output=True)
            self.assertEqual(a.returncode,0,a.stdout+a.stderr)
            self.assertEqual(b.returncode,2,b.stdout+b.stderr)
            self.assertFalse(json.loads(b.stdout)["target_met"])
            one=base/"one/reports"; two=base/"two/reports"
            for path in one.iterdir():self.assertEqual(path.read_bytes(),(two/path.name).read_bytes())
            manifest=read_json(one/"dimension_expansion_manifest.json")
            for name,meta in manifest["files"].items():
                payload=(one/name).read_bytes()
                self.assertEqual(meta["sha256"],hashlib.sha256(payload).hexdigest())
                self.assertEqual(meta["size_bytes"],len(payload))


if __name__ == "__main__":
    unittest.main()
