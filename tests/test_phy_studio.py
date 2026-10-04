"""Source fidelity, bilateral closure and manufacturing-file integrity for Studio."""
from collections import Counter
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from skeleton.visualization import studio_model as sm

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("studio_export", ROOT / "bin/export_phy_studio.py")
exporter = importlib.util.module_from_spec(spec); spec.loader.exec_module(exporter)


class PHYStudioTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = sm.build_armature()

    def test_exact_age28_cohort_and_known_means(self):
        ref = sm.reference_statistics()
        self.assertEqual((ref["n"], ref["age_years"]), (92, 28))
        self.assertAlmostEqual(ref["statistics"]["stature"]["mean_mm"], 1632.4021739130435, places=5)
        self.assertAlmostEqual(ref["statistics"]["span"]["mean_mm"], 1664.358695652174, places=5)
        self.assertIn("not a representative", ref["population"].lower())

    def test_tampered_source_fails_closed(self):
        original = sm.read
        def altered(path):
            data = original(path)
            if Path(path).name == "ansur_age28.json":
                data["rows"][0][1] += 1
            return data
        with patch.object(sm, "read", altered), self.assertRaisesRegex(ValueError, "hash mismatch"):
            sm.reference_statistics()

    def test_scale_span_and_floor_vertex_close(self):
        for height, constrained in ((None, False), (1676.4, True), (1800, False)):
            with self.subTest(height=height):
                model = sm.build_armature(height_mm=height, arm_drop_deg=0, span_equals_height=constrained)
                self.assertAlmostEqual(model["landmarks"]["vertex"][2], model["height_mm"])
                self.assertAlmostEqual(math.dist(model["landmarks"]["fingertip_L"], model["landmarks"]["fingertip_R"]), model["t_pose_span_mm"])
                if constrained:
                    self.assertEqual(model["height_mm"], model["t_pose_span_mm"])
                vertices = [v for p in model["parts"] if p["region"] != "envelope" for v in p["vertices"]]
                self.assertAlmostEqual(min(v[2] for v in vertices), 0)
                self.assertAlmostEqual(max(v[2] for v in vertices), model["height_mm"], places=4)

    def test_bilateral_joint_centers_are_mirrors(self):
        points = self.model["landmarks"]
        for key, point in points.items():
            if key.endswith("_R"):
                self.assertEqual(points[key[:-1]+"L"], [-point[0], point[1], point[2]])
        parts = {p["id"]: p for p in self.model["parts"]}
        for key, p in parts.items():
            if key.endswith("_R") and key[:-1]+"L" in parts:
                mirrored = sorted(tuple(round(x, 4) for x in [-v[0], v[1], v[2]]) for v in p["vertices"])
                actual = sorted(tuple(round(x, 4) for x in v) for v in parts[key[:-1]+"L"]["vertices"])
                self.assertEqual(mirrored, actual, key)

    def test_every_reference_mesh_is_closed_and_has_positive_volume(self):
        for p in self.model["parts"]:
            with self.subTest(part=p["id"]):
                edges = Counter(tuple(sorted((f[i], f[(i+1) % 3]))) for f in p["faces"] for i in range(3))
                self.assertTrue(all(count == 2 for count in edges.values()))
                volume = 0
                for face in p["faces"]:
                    a, b, c = [p["vertices"][i] for i in face]
                    n = sm.cross(sm.sub(b, a), sm.sub(c, a))
                    self.assertGreater(sum(x*x for x in n), 1e-10)
                    volume += sum(a[i]*sm.cross(b, c)[i] for i in range(3))/6
                self.assertGreater(volume, 0)

    def test_refinement_bounds_and_invalid_inputs(self):
        self.assertEqual(self.model["aesthetic_adjustments_percent"], {"shoulder": -1, "waist": -3, "hip": 2})
        self.assertFalse(self.model["fabrication_released"])
        self.assertEqual(self.model["canon_effect"], "none")
        for kwargs in ({"height_mm": float("nan")}, {"height_mm": True}, {"height_mm": 2000}, {"refinement": 2}, {"arm_drop_deg": -1}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                sm.build_armature(**kwargs)

    def test_maquette_dimensions_paths_and_bom_quantities(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)
            result = exporter.maquette(self.model, path)
            self.assertAlmostEqual(result["height_mm"], self.model["height_mm"]*.25)
            self.assertEqual(result["cut_parts"], sum(r["qty"] for r in result["bill_of_materials"] if r["stock"] == "3 mm birch plywood"))
            sheets = list(path.glob("cut-sheet-*.svg"))
            self.assertEqual(len(sheets), result["cut_sheets"])
            for sheet in sheets:
                root = ET.fromstring(sheet.read_text())
                self.assertEqual(root.attrib["width"], "420mm")
                self.assertEqual(root.attrib["height"], "297mm")
                self.assertIsNotNone(root.find("{http://www.w3.org/2000/svg}g[@id='CUT']"))
            self.assertEqual(len({r["id"] for r in result["bill_of_materials"]}), len(result["bill_of_materials"]))
            self.assertIn("not a load-bearing", (path / "BUILD.md").read_text())

    @unittest.skipUnless(importlib.util.find_spec("cadquery"), "optional CadQuery unavailable")
    def test_a0_viewer_uses_existing_exact_located_geometry(self):
        model = exporter.a0_model()
        self.assertEqual(len(model["parts"]), 35)
        self.assertEqual(len({p["part_id"] for p in model["parts"]}), 19)
        existing = json.loads((ROOT / "PROJECTS/T56_CARBON/fabrication/a0_packet/assembly/assembly_instances.json").read_text())
        old = {p["instance_id"]: p for p in existing["instances"]}
        for p in model["parts"]:
            self.assertEqual(p["bounds_mm"], old[p["id"]]["bounds_mm"])
        self.assertFalse(model["fabrication_released"])
        self.assertFalse(model["physical_evidence_complete"])


if __name__ == "__main__":
    unittest.main()
