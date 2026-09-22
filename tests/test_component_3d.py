"""Source fidelity and nominal CAD checks, not physical release evidence."""
import importlib.util
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
loader = importlib.util.spec_from_file_location("component_3d", ROOT / "bin/export_component_3d.py")
module = importlib.util.module_from_spec(loader)
loader.loader.exec_module(module)


class SourceGeometryTests(unittest.TestCase):
    def test_source_units_and_missing_axes(self):
        specs, omitted = module.component_specs()
        rows = {s["id"]: s for s in specs}
        self.assertEqual(rows["REF_FEMUR"]["xyz_mm"], [40, 40, 420])
        self.assertEqual(rows["REF_SCAPULA"]["xyz_mm"], [105, 3, 145])
        self.assertEqual(rows["REDWOOD_FEMUR_BLANK"]["xyz_mm"], [44, 44, 430])
        self.assertEqual({r["id"] for r in omitted}, {"Skull", "Mandible"})
        self.assertTrue(all(s["assembly_transform"] is None for s in specs))
        self.assertTrue(all(s["source"] and s["authority"] for s in specs))

    def test_no_implicit_t56_geometry_closure(self):
        register = module.read(module.REGISTER)
        before = json.dumps(register, sort_keys=True)
        module.component_specs()
        self.assertEqual(json.dumps(module.read(module.REGISTER), sort_keys=True), before)
        self.assertEqual(sum(r["value"] is None for r in register["parameters"]), 19)

    @unittest.skipUnless(importlib.util.find_spec("cadquery"), "optional CadQuery unavailable")
    def test_bushing_keeps_axial_bore(self):
        import cadquery as cq
        specs, _ = module.component_specs()
        for spec in specs:
            if spec["kind"] != "bushing":
                continue
            shape = module.build_shape(spec)
            inner, outer, length = spec["id_od_length_mm"]
            probe = cq.Workplane("XY").circle(inner / 4).extrude(length).val()
            self.assertAlmostEqual(shape.intersect(probe).Volume(), 0, places=9)
            self.assertTrue(shape.isValid())


if __name__ == "__main__":
    unittest.main()
