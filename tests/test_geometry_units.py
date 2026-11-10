"""Runtime primitives respect BoneSpec units and preserve spinal unknowns."""
from copy import deepcopy
import unittest
import warnings

from geometry.geometry_agent import GeometryAgent
from skeleton.base import BoneSpec
from skeleton.bones import load_field


def fixture(dimensions, bone_type="irregular"):
    return BoneSpec(name="Unit fixture", unique_id="UNIT_FIXTURE", bone_type=bone_type,
                    location={}, articulations=[], dimensions=dimensions, function=[],
                    notable_features=[], developmental_notes="synthetic, not osteometry", variations="")


class GeometryUnitTest(unittest.TestCase):
    def test_equivalent_units_and_mixed_aliases_produce_the_same_primitives(self):
        maps = [
            {"length_cm": 10, "width_cm": 11, "thickness_cm": 1},
            {"length_mm": 100, "width_mm": 110, "thickness_mm": 10},
            {"length_m": .1, "width_m": .11, "thickness_m": .01},
            {"length_mm": 100, "width_cm": 11, "thickness_m": .01},
        ]
        for shape in ("irregular", "flat", "long"):
            baseline = fixture(maps[0], shape)
            GeometryAgent(baseline).compute()
            for dimensions in maps[1:]:
                with self.subTest(shape=shape, dimensions=dimensions):
                    bone = fixture(deepcopy(dimensions), shape)
                    original = bone.to_fabrication_record()
                    GeometryAgent(bone).compute()
                    self.assertEqual(bone.geometry, baseline.geometry)
                    self.assertEqual(bone.dimensions, dimensions)
                    self.assertEqual(bone.to_fabrication_record(), original)
        box = fixture(maps[1])
        GeometryAgent(box).compute()
        self.assertAlmostEqual(box.geometry["V_cm3"], 110)
        self.assertEqual(box.geometry["COM"], (.055, .005, .05))
        self.assertAlmostEqual(box.geometry["inertia_kgm2"][0][0], .00016665)

    def test_conflicting_aliases_are_rejected_in_both_orders(self):
        for first, second in ((10, 101), (None, 100), (10, None)):
            items = [("length_cm", first), ("length_mm", second), ("width_mm", 10), ("thickness_mm", 5)]
            for ordered in (items, list(reversed(items))):
                with self.subTest(ordered=ordered):
                    bone = fixture(dict(ordered))
                    with self.assertRaisesRegex(ValueError, "Conflicting dimension aliases"):
                        GeometryAgent(bone).compute()
                    self.assertEqual(bone.geometry, {})

    def test_invalid_sizes_never_create_degenerate_or_nonfinite_primitives(self):
        for value in (0, -1, float("nan"), float("inf"), True, "10"):
            for key in ("length_mm", "width_mm", "thickness_mm"):
                with self.subTest(value=value, key=key):
                    dimensions = dict(length_mm=100, width_mm=110, thickness_mm=10)
                    dimensions[key] = value
                    bone = fixture(dimensions)
                    with self.assertRaises((ValueError, TypeError)):
                        GeometryAgent(bone).compute()
                    self.assertEqual(bone.geometry, {})

    def test_real_spinal_body_and_canal_features_do_not_become_solid_envelopes(self):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            field = load_field()
        for uid in [f"BONE_{p}{i}" for p, n in (("C", 7), ("T", 12), ("L", 5)) for i in range(1, n + 1)] + ["BONE_COCCYX"]:
            with self.subTest(bone=uid):
                bone = field.get(uid)
                before = bone.to_fabrication_record()
                bone.geometry["runtime_stale_marker"] = True
                GeometryAgent(bone).recompute()
                self.assertEqual(bone.geometry, {})
                self.assertEqual(bone.to_fabrication_record(), before)
                self.assertIsNone(before["physics"]["mass_kg"])


if __name__ == "__main__":
    unittest.main()
