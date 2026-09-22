"""Focused A0 article geometry and fabrication-packet checks."""
import csv
import importlib.util
import json
import math
from pathlib import Path
import unittest

from PROJECTS.T56_CARBON.tools.shoulder_dimensions import review

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "PROJECTS" / "T56_CARBON"
DESIGN = PROJECT / "requirements" / "a0_shoulder_mechanism.json"
PACKET = PROJECT / "fabrication" / "a0_packet"
spec = importlib.util.spec_from_file_location("a0_export", ROOT / "bin/export_t56_a0_article.py")
exporter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(exporter)


class T56A0GeometryTest(unittest.TestCase):
    def load(self, path):
        return json.loads(path.read_text(encoding="utf-8"))

    def test_parallel_four_bar_and_trajectory_are_consistent(self):
        design = self.load(DESIGN)
        stage = design["mechanism"]["scapular_stage"]
        p, a = stage["posterior_root_mm"], stage["anterior_root_mm"]
        b, c = stage["posterior_carriage_pivot_mm"], stage["anterior_carriage_pivot_mm"]
        distance = lambda u, v: math.dist(u, v)
        self.assertAlmostEqual(distance(p, b), 55)
        self.assertAlmostEqual(distance(a, c), 55)
        self.assertAlmostEqual(distance(p, a), 100)
        self.assertAlmostEqual(distance(b, c), 100)
        self.assertEqual(stage["range_deg"], stage["hard_stop_range_deg"])
        expected_y = [-10 - 55 * math.sin(math.radians(10)), -10,
                      -10 + 55 * math.sin(math.radians(10))]
        for row, y in zip(design["trajectory"], expected_y):
            self.assertAlmostEqual(row["position_mm"][1], y, delta=0.001)

    def test_member_resolver_closes_exact_declared_stack(self):
        inputs = self.load(PROJECT / "requirements" / "shoulder_member_inputs.json")
        result = review(inputs)
        self.assertEqual(result["status"], "dimensional_review_only")
        self.assertEqual(result["dimensions_mm"], {
            "effective_length": 317, "seat_gap": 267, "stock_cut_length": 327,
        })
        self.assertEqual(result["worst_case_bounds_mm"], {
            "effective_length": 1.0, "seat_gap": 1.5, "stock_cut_length": 2.0,
        })
        self.assertFalse(result["fabrication_ready"])
        self.assertFalse(result["canon_adopted"])

    def test_register_uses_design_values_without_canon_claim(self):
        design = self.load(DESIGN)
        register = self.load(PROJECT / "requirements" / "geometry_register.json")
        rows = {row["id"]: row for row in register["parameters"]}
        self.assertEqual(design["canon_effect"], "none")
        self.assertEqual(rows["GEO-SHO-006"]["value"], design["mechanism"]["humeral_stage"]["joint_center_mm"])
        self.assertEqual(rows["GEO-ARM-001"]["value"], design["dummy_member"]["effective_length_mm"])
        self.assertTrue(all(rows[item]["status"] == "locked" for item in (
            "GEO-SHO-001", "GEO-SHO-002", "GEO-SHO-003", "GEO-SHO-004",
            "GEO-SHO-005", "GEO-SHO-006", "GEO-SHO-007", "GEO-ARM-001",
            "GEO-CLR-001", "GEO-CLR-002", "GEO-CLR-003",
        )))

    def test_packet_is_explicitly_unreleased_and_bom_is_unique(self):
        manifest = self.load(PACKET / "manifest.json")
        self.assertFalse(manifest["fabrication_released"])
        self.assertFalse(manifest["physical_evidence_complete"])
        with (PACKET / "BOM.csv").open(newline="", encoding="utf-8") as handle:
            rows = list(csv.DictReader(handle))
        self.assertEqual(len(rows), manifest["bom_items"])
        self.assertGreaterEqual(len(rows), 18)
        self.assertEqual(len({row["item_id"] for row in rows}), len(rows))
        for name in manifest["drawing_files"]:
            drawing = (PACKET / "drawings" / name).read_text(encoding="utf-8")
            self.assertIn("NOT FABRICATION RELEASED", drawing)

    @unittest.skipUnless(importlib.util.find_spec("cadquery"), "optional CadQuery unavailable")
    def test_all_part_models_are_single_valid_solids(self):
        for name, workplane in exporter.build_parts().items():
            with self.subTest(part=name):
                shape = workplane.val()
                self.assertTrue(shape.isValid())
                self.assertEqual(len(shape.Solids()), 1)
                self.assertGreater(shape.Volume(), 0)


if __name__ == "__main__":
    unittest.main()
