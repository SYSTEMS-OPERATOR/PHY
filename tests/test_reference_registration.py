"""Reference frame fits, custody and rejection of unsupported adoption."""
from copy import deepcopy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

from skeleton.reference_registration import register_landmarks
from skeleton.validation.reference_correspondence import audit_correspondence

ROOT = Path(__file__).resolve().parents[1]


def packet():
    source = {"a": [0, 0, 0], "b": [0.1, 0, 0], "c": [0, 0.2, 0], "d": [0, 0, 0.3]}
    # Explicit proper rotation and translation, independent of the fitter.
    target = {k: [-1000*v[1]+17, 1000*v[0]-23, 1000*v[2]+41] for k,v in source.items()}
    return {"source": {"frame": "native-test", "units": "m", "points": source},
            "target": {"frame": "review-test", "units": "mm", "points": target},
            "max_residual_mm": 1e-6}


class RegistrationTests(unittest.TestCase):
    def test_proper_rigid_fit_uses_only_explicit_unit_scale(self):
        result = register_landmarks(packet())
        np.testing.assert_allclose(result["rotation"], [[0,-1,0],[1,0,0],[0,0,1]], atol=1e-12)
        np.testing.assert_allclose(result["translation_mm"], [17,-23,41], atol=1e-12)
        self.assertAlmostEqual(result["determinant"], 1)
        self.assertEqual(result["fitted_scale"], 1)
        self.assertEqual(result["source_units"], "m")
        self.assertEqual(result["target_units"], "mm")
        self.assertEqual(result["source_to_mm_scale"], 1000)
        self.assertFalse(result["anatomically_qualified"])
        self.assertFalse(result["fabrication_released"])
        self.assertEqual(result["canon_effect"], "none")

    def test_reflection_scaling_and_bad_landmarks_fail_residual_gate(self):
        for mode in ("reflect", "scale", "displace"):
            p = packet()
            for value in p["target"]["points"].values():
                if mode == "reflect": value[0] *= -1
                elif mode == "scale": value[:] = [x*1.05 for x in value]
            if mode == "displace": p["target"]["points"]["d"][0] += 10
            with self.subTest(mode=mode), self.assertRaisesRegex(ValueError, "residual"):
                register_landmarks(p)

    def test_coplanar_unknown_units_invalid_names_and_nonfinite_fail(self):
        changes = [lambda p:p["source"]["points"]["d"].__setitem__(2,0),
                   lambda p:p["source"].__setitem__("units","cm"),
                   lambda p:p["source"]["points"].__setitem__(1,[0,0,0]),
                   lambda p:p["source"]["points"]["a"].__setitem__(0,float("nan")),
                   lambda p:p["source"]["points"]["a"].__setitem__(0,1e308),
                   lambda p:p["source"].__setitem__("frame",p["target"]["frame"]),
                   lambda p:p["target"]["points"].pop("a")]
        for change in changes:
            p=packet();change(p)
            with self.subTest(change=change), self.assertRaises(ValueError): register_landmarks(p)

    def test_cli_rejects_duplicate_json_and_preserves_previous_output(self):
        with tempfile.TemporaryDirectory() as d:
            source=Path(d)/"input.json";output=Path(d)/"output.json"
            source.write_text(json.dumps(packet()))
            command=[sys.executable,str(ROOT/"bin/register_reference_landmarks.py"),
                     "--input",str(source),"--output",str(output)]
            self.assertEqual(subprocess.run(command,capture_output=True).returncode,0)
            before=output.read_bytes()
            source.write_text('{"source":{},"source":{}}')
            self.assertEqual(subprocess.run(command,capture_output=True).returncode,1)
            self.assertEqual(output.read_bytes(),before)
            self.assertEqual(sorted(p.name for p in Path(d).iterdir()),["input.json","output.json"])


class CorrespondenceTests(unittest.TestCase):
    def fixture(self):
        audit={"asset_id":"NIH_TEST", "sha256":"a"*64,
            "coordinate_contract":"glTF right-handed Y-up; linear units meters",
            "anatomical_axis_registration":None,"anatomically_qualified":False,
            "fabrication_released":False,"nodes":[
                {"name":"sacrum","index":2,"mesh":0,"triangles":8,"in_default_scene":True},
                {"name":"extra_lumbar","index":3,"mesh":1,"triangles":8,"in_default_scene":True}]}
        catalog={"authority":"curated_reference_correspondence_not_geometry_adoption",
            "canonical_transform":None,"fabrication_released":False,
            "source_revision":"c"*40,"source_asset":"NIH_TEST","source_sha256":"a"*64,
            "audit_sha256":"b"*64,"entries":[{"bone_id":"BONE_SACRUM",
                "source_nodes":["sacrum"],"mapping_basis":"Identity label only"}],
            "unmapped_source_review":[{"source_node":"extra_lumbar","reason":"Hold variant"}]}
        return catalog,audit

    def test_labels_match_without_dimensions_or_variant_adoption(self):
        c,a=self.fixture();r=audit_correspondence(c,a,"b"*64,{"BONE_SACRUM"})
        self.assertEqual(r["matched_canonical_ids"],1)
        self.assertEqual(r["unmapped_source_review"][0]["source_node"],"extra_lumbar")
        self.assertFalse(r["rows"][0]["canonical_dimensions_adopted"])
        self.assertIsNone(r["canonical_transform"])

    def test_changed_hash_units_duplicate_labels_and_invalid_ids_are_refused(self):
        for kind in ("hash","units","label","duplicate","identity","variant","transform"):
            c,a=self.fixture()
            if kind=="hash": a["sha256"]="0"*64
            elif kind=="units":a["coordinate_contract"]="mm"
            elif kind=="label":a["nodes"][0]["name"]="wrong"
            elif kind=="duplicate":a["nodes"].append(deepcopy(a["nodes"][0]))
            elif kind=="identity":c["entries"][0]["bone_id"]="BONE_L6"
            elif kind=="variant":c["entries"][0]["source_nodes"]=["extra_lumbar"]
            elif kind=="transform":c["canonical_transform"]=[1,0,0]
            with self.subTest(kind=kind),self.assertRaises(ValueError):
                audit_correspondence(c,a,"b"*64,{"BONE_SACRUM"})

    def test_nasa_catalog_retains_conditions_and_source_ambiguities(self):
        d=json.loads((ROOT/"references/nasa_positioning_ranges.json").read_text())
        self.assertEqual(len(d["ranges"]),52)
        self.assertEqual(len(d["anthropometry"]),11)
        self.assertEqual({r["condition"] for r in d["ranges"]},
                         {"unsuited","suited_unpressurized","suited_pressurized"})
        self.assertTrue(all(r["mechanical_axis"] is None and r["mechanical_limit_deg"] is None
                            and r["combined_joint_feasibility"] is None for r in d["ranges"]))
        self.assertEqual(sum(bool(r["source_issue"]) for r in d["ranges"]),17)
        self.assertFalse(d["fabrication_released"])


if __name__ == "__main__": unittest.main()
