"""Rendering links must preserve identity without guessing a bilateral side."""
import unittest

from skeleton.base import BoneSpec
from skeleton.field import SkeletonField


def bone(uid, name, *references):
    return BoneSpec(
        unique_id=uid, name=name, bone_type="long", location={"region": "test"},
        articulations=[{"bone": reference} for reference in references],
        dimensions={}, function=[], notable_features=[],
        developmental_notes="synthetic rendering fixture", variations="",
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


if __name__ == "__main__":
    unittest.main()
