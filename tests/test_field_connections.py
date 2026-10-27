"""Rendering and signal links must preserve identity without guessing a side."""
import unittest

from skeleton.base import BoneSpec
from skeleton.field import SkeletonField


def bone(uid, name, *references):
    return BoneSpec(
        unique_id=uid, name=name, bone_type="long", location={"region": "test"},
        articulations=[{"bone": reference} for reference in references],
        dimensions={}, function=[], notable_features=[],
        developmental_notes="synthetic connection fixture", variations="",
    )


class FieldConnectionTest(unittest.TestCase):
    def test_exact_bone_id_selects_the_correct_bilateral_target(self):
        field = SkeletonField([
            bone("FEMUR_L", "Femur"), bone("FEMUR_R", "Femur"),
            bone("TIBIA_L", "Tibia", "FEMUR_L"),
        ])
        self.assertEqual(field._connection_pairs(), [("TIBIA_L", "FEMUR_L")])

    def test_ambiguous_display_name_never_guesses_from_registration_order(self):
        for order in (("L", "R"), ("R", "L")):
            with self.subTest(order=order):
                field = SkeletonField([
                    *(bone("FEMUR_"+side, "Femur") for side in order),
                    bone("TIBIA_L", "Tibia", "Femur"),
                ])
                self.assertEqual(field._connection_pairs(), [])

    def test_exact_id_takes_precedence_over_matching_display_names(self):
        field = SkeletonField([
            bone("Femur", "Other display name"),
            bone("FEMUR_L", "Femur"), bone("FEMUR_R", "Femur"),
            bone("TIBIA_L", "Tibia", "Femur"),
        ])
        self.assertEqual(field._connection_pairs(), [("TIBIA_L", "Femur")])

    def test_unique_display_names_still_resolve_and_missing_targets_stay_absent(self):
        field = SkeletonField([
            bone("PELVIS", "Pelvis"),
            bone("FEMUR_L", "Femur", "Pelvis", "Missing target"),
        ])
        self.assertEqual(field._connection_pairs(), [("FEMUR_L", "PELVIS")])


class ArticulationSignalTest(unittest.TestCase):
    def test_exact_bone_id_wires_and_signals_only_the_selected_side(self):
        left, right = bone("FEMUR_L", "Femur"), bone("FEMUR_R", "Femur")
        source = bone("TIBIA_L", "Tibia", "FEMUR_L")
        left.domain_id = "runtime-left"
        field = SkeletonField([left, right, source])
        field.propagate(source.domain_id, {"voltage": 4.0})
        self.assertEqual(source.entanglement_links, [left.domain_id])
        self.assertEqual(left.entanglement_links, [source.domain_id])
        self.assertEqual(right.entanglement_links, [])
        self.assertEqual((source.voltage_potential, left.voltage_potential, right.voltage_potential),
                         (0.0, 2.0, 0.0))

    def test_ambiguous_names_do_not_fan_out_to_both_sides(self):
        for order in (("L", "R"), ("R", "L")):
            with self.subTest(order=order):
                targets = [bone("FEMUR_"+side, "Femur") for side in order]
                source = bone("TIBIA_L", "Tibia", "Femur")
                field = SkeletonField(targets+[source])
                field.propagate(source.domain_id, {"voltage": 4.0})
                self.assertTrue(all(not b.entanglement_links for b in targets+[source]))
                self.assertTrue(all(b.voltage_potential == 0.0 for b in targets))

    def test_exact_id_wins_over_display_name_collisions(self):
        selected = bone("Femur", "Other display name")
        left, right = bone("FEMUR_L", "Femur"), bone("FEMUR_R", "Femur")
        source = bone("TIBIA_L", "Tibia", "Femur")
        field = SkeletonField([selected, left, right, source])
        field.propagate(source.domain_id, {"voltage": 4.0})
        self.assertEqual(source.entanglement_links, [selected.domain_id])
        self.assertEqual((selected.voltage_potential, left.voltage_potential, right.voltage_potential),
                         (2.0, 0.0, 0.0))

    def test_unique_legacy_names_remain_case_insensitive_and_rebuild_is_idempotent(self):
        target = bone("PELVIS", "Pelvis")
        source = bone("FEMUR_L", "Femur", "pelvis", "Missing target")
        field = SkeletonField([target, source])
        field.build_entanglement()
        self.assertEqual(source.entanglement_links, [target.domain_id])
        self.assertEqual(target.entanglement_links, [source.domain_id])
        field.propagate(source.domain_id, {"voltage": 4.0})
        self.assertEqual(target.voltage_potential, 2.0)

    def test_case_insensitive_name_collisions_stay_unresolved(self):
        left, right = bone("FEMUR_L", "Femur"), bone("FEMUR_R", "FEMUR")
        source = bone("TIBIA_L", "Tibia", "femur")
        SkeletonField([left, right, source])
        self.assertTrue(all(not b.entanglement_links for b in (left, right, source)))


if __name__ == "__main__":
    unittest.main()
