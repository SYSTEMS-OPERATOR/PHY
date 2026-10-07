"""Direct fabrication-gate regressions; fixtures are software evidence only."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
import warnings

from skeleton.base import BoneSpec
from skeleton.bones import load_field
from skeleton.field import SkeletonField
from skeleton.validation.validator_agent import ValidatorAgent


def fixture(uid="REVIEW_FIXTURE", **changes):
    values = dict(
        name=uid, bone_type="long", location={"region": "test"}, articulations=[],
        dimensions={"length_mm": 100.0, "width_mm": 10.0, "thickness_mm": 10.0},
        function=[], notable_features=[], developmental_notes="synthetic", variations="",
        unique_id=uid, geometry={"shape": "box"},
        material={"name": "synthetic", "density": 1800.0}, physics={"mass_kg": 0.018},
        joint_interfaces=[{"name": "synthetic_joint"}],
        mount_points=[{"name": "synthetic_mount"}],
        manufacturing_notes=["Synthetic fixture, not a released part"],
        references=["synthetic-test-evidence"],
    )
    values.update(changes)
    return BoneSpec(**values)


class FabricationValidationTest(unittest.TestCase):
    def report(self, bone):
        return ValidatorAgent(SkeletonField([bone])).run()

    def assert_blocked(self, report, category):
        self.assertFalse(report["summary"]["pass"])
        self.assertFalse(report["export_readiness"]["ready"])
        self.assertTrue(report[category], category)
        self.assertIn(category, report["export_readiness"]["issues"])
        # Every invalid case must still yield interoperable JSON.
        json.dumps(report, allow_nan=False)

    def test_valid_positive_control_and_cm_conversion(self):
        for dims in ({"length_mm": 100, "width_mm": 10, "thickness_mm": 10},
                     {"length_cm": 10, "width_cm": 1, "thickness_cm": 1}):
            with self.subTest(dimensions=dims):
                bone = fixture(dimensions=dims)
                report = self.report(bone)
                self.assertTrue(report["summary"]["pass"])
                self.assertTrue(report["export_readiness"]["ready"])
                self.assertEqual(report["export_readiness"]["issues"], [])
                self.assertEqual(bone.to_fabrication_record()["dimensions"],
                                 {"length_mm": 100.0, "width_mm": 10.0, "thickness_mm": 10.0})

    def test_unknown_empty_and_mixed_dimensions_block(self):
        for dims in ({}, {"length_mm": None},
                     {"length_mm": 100, "width_mm": None},
                     {"length_mm": None, "width_mm": None, "thickness_mm": None}):
            with self.subTest(dimensions=dims):
                self.assert_blocked(self.report(fixture(dimensions=dims)), "missing_dimension_values")

    def test_invalid_dimension_values_are_structured_blockers(self):
        for value in (0, -1, float("nan"), float("inf"), float("-inf"),
                      True, False, "10", "TBD", [], {}, 10**400):
            with self.subTest(value=repr(value)):
                self.assert_blocked(self.report(fixture(dimensions={"length_mm": value})),
                                    "impossible_geometry_values")
        self.assert_blocked(self.report(fixture(dimensions={"length_cm": 1e308})),
                            "impossible_geometry_values")

    def test_invalid_density_and_mass_are_structured_blockers(self):
        for density in (None, 0, -1, float("nan"), float("inf"), True, "1800", [], 10**400):
            with self.subTest(density=repr(density)):
                self.assert_blocked(self.report(fixture(material={"density": density})),
                                    "material_incompleteness")
        self.assert_blocked(self.report(fixture(material={})), "material_incompleteness")
        self.assert_blocked(self.report(fixture(material={"density": None}, physics={})),
                            "record_conversion_errors")
        for mass in (None, 0, -1, 100.01, float("nan"), float("inf"), True, "1", [], 10**400):
            with self.subTest(mass=repr(mass)):
                self.assert_blocked(self.report(fixture(physics={"mass_kg": mass})),
                                    "mass_inertia_plausibility")

    def test_all_four_canonical_units_are_checked(self):
        for unit, wrong in (("length", "cm"), ("mass", "g"),
                            ("density", "g/cm^3"), ("inertia", "g*cm^2")):
            for missing in (False, True):
                bone = fixture()
                if missing:
                    bone.units.pop(unit)
                else:
                    bone.units[unit] = wrong
                with self.subTest(unit=unit, missing=missing):
                    self.assert_blocked(self.report(bone), "invalid_unit_combinations")

    def test_empty_or_malformed_interfaces_are_blocked(self):
        for attr, category in (("joint_interfaces", "missing_joint_interfaces"),
                               ("mount_points", "missing_mount_points")):
            for value in ([], [{}], [None], ["TBD"], {"name": "wrong_container"}):
                bone = fixture()
                setattr(bone, attr, value)
                with self.subTest(field=attr, value=value):
                    self.assert_blocked(self.report(bone), category)

    def test_malformed_containers_report_instead_of_raising(self):
        for attr in ("dimensions", "units", "geometry", "material", "physics",
                     "connections", "tolerance", "references", "manufacturing_notes", "source_ids"):
            for value in (None, "TBD", 12):
                bone = fixture()
                setattr(bone, attr, value)
                with self.subTest(field=attr, value=value):
                    self.assert_blocked(self.report(bone), "invalid_field_types")

    def test_nested_nonfinite_geometry_and_invalid_tolerance_are_blocked(self):
        bone = fixture(geometry={"shape": "box", "origin_mm": [0, float("nan"), 0]})
        self.assert_blocked(self.report(bone), "impossible_geometry_values")
        for value in (None, -0.1, float("inf"), True, "0.5"):
            with self.subTest(tolerance=value):
                self.assert_blocked(self.report(fixture(tolerance={"default_mm": value})),
                                    "invalid_tolerances")

    def test_connections_are_validated_before_reference_lookup(self):
        for value in ({"parent": []}, {"children": "REVIEW_FIXTURE"},
                      {"children": [None]}, {"parent": "MISSING"}, {"children": ["MISSING"]}):
            with self.subTest(connections=value):
                self.assert_blocked(self.report(fixture(connections=value)),
                                    "invalid_parent_child_references")
        parent, child = fixture("PARENT"), fixture("CHILD")
        parent.connections = {"parent": None, "children": [child.unique_id]}
        child.connections = {"parent": parent.unique_id, "children": []}
        self.assertTrue(ValidatorAgent(SkeletonField([parent, child])).run()["summary"]["pass"])

    def test_empty_skeleton_fails_without_requiring_a_whole_body_for_positive_control(self):
        self.assert_blocked(ValidatorAgent(SkeletonField()).run(), "empty_skeleton")
        self.assertTrue(self.report(fixture())["summary"]["pass"])

    def test_validation_detects_identity_collisions_after_registration(self):
        a, b = fixture("A"), fixture("B")
        field = SkeletonField([a, b])
        b.unique_id = a.unique_id
        self.assert_blocked(ValidatorAgent(field).run(), "duplicate_bone_ids")

    def test_missing_dataset_binding_is_checked_on_the_source_record(self):
        bone = fixture()
        bone.dataset = {}
        self.assert_blocked(self.report(bone), "dataset_schema_mismatch")
        bone.dataset_key = "SYNTHETIC"
        self.assert_blocked(self.report(bone), "dataset_schema_mismatch")
        bone.dataset = {"SYNTHETIC": {}}
        self.assertTrue(self.report(bone)["summary"]["pass"])

    def test_mass_export_and_validation_are_pure_and_embodiment_independent(self):
        bone = fixture(physics={})
        before = copy.deepcopy(bone.__dict__)
        virtual_record = bone.to_fabrication_record()
        virtual_report = self.report(bone)
        self.assertEqual(bone.__dict__, before)
        self.assertAlmostEqual(virtual_record["physics"]["mass_kg"], 0.018)
        bone.embodiment = "physical"
        self.assertEqual(bone.to_fabrication_record(), virtual_record)
        self.assertEqual(self.report(bone), virtual_report)
        self.assertTrue(virtual_report["summary"]["pass"])

    def test_conversion_error_is_reported_and_other_bones_are_still_checked(self):
        bone = fixture("CONVERSION_ERROR")
        bone.to_fabrication_record = lambda: (_ for _ in ()).throw(ValueError("bad conversion"))
        other = fixture("UNKNOWN", dimensions={"length_mm": None})
        report = ValidatorAgent(SkeletonField([bone, other])).run()
        self.assert_blocked(report, "record_conversion_errors")
        self.assertEqual(report["summary"]["total_bones"], 2)
        self.assertTrue(report["missing_dimension_values"])

    def test_report_order_and_serialization_are_deterministic(self):
        a = fixture("A", dimensions={"length_mm": float("inf")})
        b = fixture("B", physics={"mass_kg": float("nan")})
        first = ValidatorAgent(SkeletonField([a, b])).run()
        second = ValidatorAgent(SkeletonField([b, a])).run()
        self.assertEqual(first, second)
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            one = ValidatorAgent.write_reports(first, root / "one")
            two = ValidatorAgent.write_reports(second, root / "two")
            for p, q in zip(one, two):
                self.assertEqual(p.read_bytes(), q.read_bytes())
            json.loads(one[0].read_text(), parse_constant=lambda value: self.fail(value))

    def test_current_model_keeps_unknown_measurements_and_stays_blocked(self):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            field = load_field()
        before = {uid: copy.deepcopy(b.dimensions) for uid, b in field.bones.items()}
        report = ValidatorAgent(field).run()
        self.assert_blocked(report, "missing_dimension_values")
        self.assertEqual(report["summary"]["total_bones"], 193)
        self.assertEqual({row["bone"] for row in report["missing_dimension_values"]},
                         {b.unique_id for b in field.bones.values()
                          if not b.dimensions or any(v is None for v in b.dimensions.values())})
        self.assertEqual({uid: b.dimensions for uid, b in field.bones.items()}, before)


class BoneRegistrationTest(unittest.TestCase):
    def test_duplicate_domain_never_overwrites_the_original(self):
        original, replacement = fixture("ORIGINAL"), fixture("REPLACEMENT")
        replacement.domain_id = original.domain_id
        field = SkeletonField([original])
        with self.assertRaisesRegex(ValueError, "domain_id"):
            field.register(replacement)
        self.assertIs(field.get(original.domain_id), original)
        self.assertEqual(len(field.bones), 1)

    def test_duplicate_bone_id_in_different_domains_is_rejected(self):
        original, replacement = fixture(), fixture()
        replacement.domain_id = "OTHER_DOMAIN"
        field = SkeletonField([original])
        with self.assertRaisesRegex(ValueError, "unique_id"):
            field.register(replacement)
        self.assertEqual(len(field.bones), 1)

    def test_constructor_and_repeated_registration_reject_duplicates(self):
        bone = fixture()
        with self.assertRaises(ValueError):
            SkeletonField([bone, bone])
        field = SkeletonField([bone])
        with self.assertRaises(ValueError):
            field.register(bone)

    def test_distinct_bones_with_the_same_display_name_are_allowed(self):
        field = SkeletonField([fixture("LEFT", name="Pair"), fixture("RIGHT", name="Pair")])
        self.assertEqual(len(field.bones), 2)

    def test_registration_rejects_missing_or_malformed_identity(self):
        for attr in ("domain_id", "unique_id"):
            for value in (None, "", " ", 1, []):
                bone = fixture()
                setattr(bone, attr, value)
                with self.subTest(field=attr, value=value), self.assertRaises(ValueError):
                    SkeletonField([bone])


if __name__ == "__main__":
    unittest.main()
