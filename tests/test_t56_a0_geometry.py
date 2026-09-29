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
            "effective_length": 317, "seat_gap": 252, "stock_cut_length": 327,
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

    @unittest.skipUnless(importlib.util.find_spec("cadquery"), "optional CadQuery unavailable")
    def test_r1_stock_thicknesses_and_key_are_not_doubled(self):
        parts = exporter.build_parts()
        yoke = parts["A0-104_yoke_side"].val().BoundingBox()
        sector = parts["A0-106_index_sector"].val().BoundingBox()
        key = parts["A0-202_key"].val().BoundingBox()
        self.assertAlmostEqual(yoke.ylen, 8.0)
        self.assertAlmostEqual(sector.ylen, 6.0)
        self.assertEqual(sorted(round(v, 6) for v in (key.xlen, key.ylen, key.zlen)), [5.0, 5.0, 30.0])

    @unittest.skipUnless(importlib.util.find_spec("cadquery"), "optional CadQuery unavailable")
    def test_neutral_member_closes_stack_without_crossing_shaft(self):
        design = self.load(DESIGN)
        parts = exporter.build_parts(design)
        instances = {iid: shape.val().BoundingBox() for iid, _, shape in exporter.located_instances(parts, design)}
        member, shaft = instances["dummy_member"], instances["shaft"]
        self.assertAlmostEqual(member.xmax, -217.0)
        self.assertAlmostEqual(member.xmin, -544.0)
        self.assertAlmostEqual(shaft.xmin, -213.5)
        self.assertGreaterEqual(shaft.xmin - member.xmax, 3.5)
        result = review(self.load(PROJECT / "requirements" / "shoulder_member_inputs.json"))
        self.assertEqual(result["points_mm"]["cut_S"], [-217.0, -10.0, 70.0])
        self.assertEqual(result["points_mm"]["cut_E"], [-544.0, -10.0, 70.0])

    @unittest.skipUnless(importlib.util.find_spec("cadquery"), "optional CadQuery unavailable")
    def test_located_assembly_has_required_interfaces_and_quantities(self):
        rows = exporter.located_instances(exporter.build_parts())
        ids = [row[0] for row in rows]
        self.assertEqual(len(ids), 35)
        self.assertEqual(len(ids), len(set(ids)))
        for required in ("root_standoff_P", "root_standoff_A", "yoke_1", "yoke_2",
                         "bearing_1", "bearing_2", "shaft", "key", "sector",
                         "bearing_cap_1", "bearing_cap_2", "friction_stack",
                         "retainer_1", "retainer_2", "index_plunger",
                         "scapular_index_plunger", "secondary_tether"):
            self.assertIn(required, ids)

    def test_sampled_motion_is_explicitly_not_continuous_proof(self):
        report = exporter.motion_report()
        self.assertEqual(report["sample_count"], 35)
        self.assertEqual(report["sampled_collision_count"], 0)
        self.assertEqual(report["result"], "pass_at_samples_only")
        self.assertFalse(report["continuous_clearance_proof"])

    def test_bom_and_drawings_are_driven_by_r1_authority(self):
        design = self.load(DESIGN)
        fab = design["fabrication_geometry"]
        rows = {row[0]: row for row in exporter.bom_rows(design)}
        self.assertIn(str(fab["cartridge"]["shaft_length"]), rows["A0-201"][4])
        self.assertIn(str(fab["dummy_member"]["cut_length"]), rows["A0-108"][4])
        self.assertEqual(len(exporter.drawings(design)), 6)
        rendered = "\n".join(exporter.drawings(design).values())
        self.assertNotIn("8 mm keyed drive interface", rendered)
        self.assertIn("17 h6", rendered)


if __name__ == "__main__":
    unittest.main()
