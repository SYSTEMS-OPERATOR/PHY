"""Direct fabrication-gate regressions; fixtures are software evidence only."""
import copy
from importlib import import_module
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import warnings

from skeleton.base import BoneSpec
from geometry.geometry_agent import GeometryAgent
from skeleton.bones import load_bones, load_field
from skeleton.exporters.exporter_agent import ExporterAgent
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
        for unit in ("cm", "m"):
            self.assert_blocked(self.report(fixture(dimensions={f"length_{unit}": 1e308})),
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


class CanonicalLoadingTest(unittest.TestCase):
    def load(self, field=False):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            return load_field() if field else load_bones()

    def test_loads_isolate_nested_data_and_runtime_state(self):
        first, second = self.load(field=True), self.load(field=True)
        a, b = first.get("BONE_FEMUR_L"), second.get("BONE_FEMUR_L")
        before = copy.deepcopy(b.__dict__)
        self.assertIsNot(a, b)
        a.material["name"] = "runtime-marker"
        a.location["region"] = "runtime-marker"
        a.geometry["origin_mm"] = [1, 2, 3]
        a.connections["children"].append("runtime-marker")
        a.dimensions["length_cm"] = 1
        a.metric_sources["length_cm"] = "runtime-marker"
        a.references.append("runtime-marker")
        a.dataset["Femur"]["length_cm"] = 1
        a.state_faults.append("runtime-marker")
        first.broadcast(10)
        first.collapse()
        self.assertEqual(b.__dict__, before)
        # Shared names do not imply shared mutable dataset tables within a load.
        right = first.get("BONE_FEMUR_R")
        self.assertEqual(right.dataset["Femur"]["length_cm"], 42)
        self.assertIsNot(a.dataset, right.dataset)

    def test_mutated_legacy_singletons_are_neither_cloned_nor_reloaded(self):
        module = import_module("skeleton.bones.bone_femur_l")
        with patch.object(module, "bone", copy.deepcopy(module.bone)):
            module.bone.material["name"] = "legacy-marker"
            module.bone.dimensions["length_cm"] = -500
            module.bone.geometry["verts"] = [[1, 2, 3]]
            module.bone.state_faults.append("legacy-marker")
            module.bone.voltage_potential = 99
            before = copy.deepcopy(module.bone.__dict__)
            fresh = self.load(field=True).get("BONE_FEMUR_L")
            self.assertIsNot(fresh, module.bone)
            self.assertEqual(fresh.material["name"], "bone")
            self.assertEqual(fresh.dimensions["length_cm"], 42)
            self.assertEqual(fresh.state_faults, [])
            self.assertEqual(fresh.voltage_potential, 0)
            self.assertEqual(fresh.to_fabrication_record()["geometry"]["length_mm"], 420)
            self.assertEqual(module.bone.__dict__, before)

    def test_discovery_order_is_independent_of_directory_enumeration(self):
        import skeleton.bones as definitions
        expected = [b.unique_id for b in self.load()]
        files = list(Path(definitions.__file__).parent.glob("*.py"))
        with patch("skeleton.bones.Path.glob", return_value=iter(reversed(files))):
            actual = [b.unique_id for b in self.load()]
        self.assertEqual(actual, expected)
        self.assertEqual(len(actual), 193)

    def test_another_dataset_load_has_no_prior_overrides(self):
        one = {"Femur": {"length_cm": 11, "width_cm": 3, "mass_g": 9, "density_kg_m3": 1100}}
        with patch("skeleton.bones.load_dataset", side_effect=[one, {}]):
            first, second = self.load(field=True), self.load(field=True)
        a, b = first.get("BONE_FEMUR_L"), second.get("BONE_FEMUR_L")
        self.assertEqual(a.dimensions["length_cm"], 11)
        self.assertEqual(b.dimensions["length_cm"], 48)
        self.assertEqual(b.material["density"], 1800)
        self.assertNotIn("mass_g", b.material)
        self.assertIsNone(b.dataset_key)
        self.assertEqual(b.metric_sources, {})

    def test_dataset_replacement_restores_source_and_invalidates_runtime_geometry(self):
        bone = fixture()
        source = copy.deepcopy(bone.__dict__)
        dataset = {bone.name: {"length_cm": 20, "width_cm": None, "mass_g": 9, "density_kg_m3": 1100}}
        bone.apply_dataset(dataset)
        self.assertNotIn("length_mm", bone.dimensions)
        self.assertEqual(bone.dimensions["length_cm"], 20)
        self.assertIsNone(bone.dimensions["width_cm"])
        dataset[bone.name]["length_cm"] = 999
        self.assertEqual(bone.dataset[bone.name]["length_cm"], 20)
        bone.set_material({"name": "runtime-marker"})
        bone.geometry["type"] = "stale-cache"
        bone.metric_sources["old"] = "stale-binding"
        bone.apply_dataset({bone.name: {"height_cm": 2}})
        self.assertEqual(bone.dimensions, {**source["dimensions"], "height_cm": 2})
        self.assertEqual(bone.material, source["material"])
        self.assertEqual(bone.geometry, source["geometry"])
        self.assertEqual(bone.metric_sources, {"height_cm": bone.name})
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            bone.apply_dataset({})
        self.assertEqual(bone.dimensions, source["dimensions"])
        self.assertIsNone(bone.dataset_key)
        self.assertEqual(bone.metric_sources, {})

    def test_constructor_dataset_keeps_a_pristine_source_baseline(self):
        bone = fixture(dataset={"REVIEW_FIXTURE": {"length_cm": 20}})
        self.assertEqual(bone.to_fabrication_record()["dimensions"]["length_mm"], 200)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            bone.apply_dataset({})
        self.assertEqual(bone.to_fabrication_record()["dimensions"]["length_mm"], 100)

    def test_all_export_bytes_repeat_after_unrelated_runtime_activity(self):
        def export(field, root):
            paths = ExporterAgent(field).export_all(root / "dist", root / "reports", root / "exports")
            return {key: Path(path).read_bytes() for key, path in paths.items()}

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            dirty = self.load(field=True)
            before = export(dirty, root / "before")
            bone = dirty.get("BONE_FEMUR_L")
            bone.set_material({"name": "runtime-marker", "density": 800})
            bone.set_embodiment("physical")
            GeometryAgent(bone).recompute()
            bone.state_faults.append("runtime-marker")
            dirty.broadcast(10)
            dirty.meta_breath("runtime-marker")
            dirty.collapse()
            after = export(self.load(field=True), root / "after")
            self.assertEqual(len(before), 7)
            self.assertEqual(after, before)


class CanonicalGeometryTest(unittest.TestCase):
    def test_conflicting_aliases_fail_in_both_orders(self):
        for pair in ((10, 999), (None, 100), (10, None)):
            aliases = [("length_cm", pair[0]), ("length_mm", pair[1])]
            for entries in (aliases, list(reversed(aliases))):
                with self.subTest(entries=entries):
                    bone = fixture(dimensions=dict(entries))
                    report = ValidatorAgent(SkeletonField([bone])).run()
                    self.assertFalse(report["summary"]["pass"])
                    self.assertIn("record_conversion_errors", report["export_readiness"]["issues"])
                    self.assertIn("Conflicting dimension aliases", report["record_conversion_errors"][0]["issue"])
                    json.dumps(report, allow_nan=False)

    def test_equivalent_aliases_have_identical_bytes_and_prefer_mm(self):
        entries = [("length_cm", 0.07), ("length_m", 0.0007), ("length_mm", 0.7),
                   ("width_cm", 1), ("thickness_mm", 10)]
        first = fixture(dimensions=dict(entries)).to_fabrication_record()
        second = fixture(dimensions=dict(reversed(entries))).to_fabrication_record()
        self.assertEqual(json.dumps(first), json.dumps(second))
        self.assertEqual(first["dimensions"]["length_mm"], 0.7)
        bone = fixture(dimensions=dict(entries))
        self.assertTrue(ValidatorAgent(SkeletonField([bone])).run()["summary"]["pass"])

    def test_unknown_cm_and_m_keys_are_normalized_without_filling(self):
        bone = fixture(dimensions={"length_cm": None, "length_mm": None,
                                   "width_m": None, "thickness_cm": 1}, physics={})
        record = bone.to_fabrication_record()
        self.assertEqual(record["dimensions"], {"length_mm": None, "width_mm": None, "thickness_mm": 10})
        self.assertIsNone(record["geometry"]["length_mm"])
        self.assertEqual(record["geometry"]["status"], "unresolved")
        self.assertIsNone(record["physics"]["mass_kg"])
        self.assertEqual(record["physics"]["mass_provenance"]["status"], "unknown")

    def test_real_femur_uses_selected_dimensions_and_preserves_source(self):
        module = import_module("skeleton.bones.bone_femur_l")
        source = copy.deepcopy(module.bone.__dict__)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            field = load_field()
        record = field.get("BONE_FEMUR_L").to_fabrication_record()
        for key, value in (("length_mm", 420), ("width_mm", 40), ("thickness_mm", 40)):
            self.assertEqual(record["dimensions"][key], value)
            self.assertEqual(record["geometry"][key], value)
        self.assertEqual(record["geometry"]["shape"], "box")
        self.assertEqual(record["geometry"]["status"], "dimensional_estimate")
        self.assertAlmostEqual(record["physics"]["mass_kg"], 1.2768)
        self.assertEqual(record["physics"]["mass_provenance"]["method"], "box_volume_times_density")
        self.assertEqual(module.bone.__dict__, source)
        for bone in field.bones.values():
            exported = bone.to_fabrication_record()
            self.assertFalse(any(key.endswith(("_cm", "_m")) for key in exported["dimensions"]))
            self.assertFalse(any(key.endswith(("_cm", "_m")) for key in exported["geometry"]))

    def test_runtime_geometry_cache_does_not_change_canonical_record(self):
        bone = fixture(dimensions={"length_cm": 10, "width_cm": 1, "thickness_cm": 1})
        before = bone.to_fabrication_record()
        GeometryAgent(bone).compute()
        self.assertEqual(bone.geometry["type"], "cylinder")
        self.assertEqual(bone.to_fabrication_record(), before)
        GeometryAgent(bone).recompute()
        self.assertNotIn("shape", bone.geometry)
        self.assertEqual(bone.to_fabrication_record(), before)

    def test_supported_primitives_and_explicit_meter_origin(self):
        for shape in ("box", "plate"):
            with self.subTest(shape=shape):
                bone = fixture(dimensions={"length_m": 0.1, "width_m": 0.01, "thickness_m": 0.01},
                               geometry={"type": shape, "length_m": 999, "origin_m": [0.001, -0.002, 0]})
                record = bone.to_fabrication_record()
                self.assertEqual(record["geometry"]["length_mm"], 100)
                self.assertEqual(record["geometry"]["origin_mm"], [1, -2, 0])
                self.assertTrue(ValidatorAgent(SkeletonField([bone])).run()["summary"]["pass"])

    def test_geometry_origin_aliases_conflict_or_normalize_deterministically(self):
        values = {"shape": "box", "origin_cm": [0.1, 0, 0], "origin_mm": [1, 0, 0]}
        first = fixture(geometry=values).to_fabrication_record()
        second = fixture(geometry=dict(reversed(list(values.items())))).to_fabrication_record()
        self.assertEqual(json.dumps(first), json.dumps(second))
        values["origin_mm"] = [999, 0, 0]
        bone = fixture(geometry=values)
        report = ValidatorAgent(SkeletonField([bone])).run()
        self.assertTrue(report["record_conversion_errors"])
        self.assertFalse(report["export_readiness"]["ready"])

    def test_unknown_invalid_and_overflowing_source_origins_are_blocked(self):
        for origin, category in (([1, None, 0], "missing_required_fields"),
                                 ([1, 2], "record_conversion_errors"),
                                 ([True, 0, 0], "record_conversion_errors"),
                                 ([1e308, 0, 0], "impossible_geometry_values")):
            with self.subTest(origin=origin):
                bone = fixture(geometry={"shape": "box", "origin_m": origin})
                report = ValidatorAgent(SkeletonField([bone])).run()
                self.assertTrue(report[category])
                self.assertFalse(report["export_readiness"]["ready"])
                json.dumps(report, allow_nan=False)

    def test_unsupported_or_incompatible_geometry_is_unresolved_and_blocked(self):
        for geometry, dimensions in (({}, None), ({"shape": "mesh", "verts": [[1, 2, 3]]}, None),
                                     ({"shape": "box", "type": "cylinder"}, None),
                                     ({"shape": "box", "verts": [[1, 2, 3]]}, None),
                                     ({"shape": "cylinder"}, {"length_mm": 100, "width_mm": 10, "thickness_mm": 20})):
            with self.subTest(geometry=geometry, dimensions=dimensions):
                changes = {"geometry": geometry}
                if dimensions is not None:
                    changes["dimensions"] = dimensions
                bone = fixture(**changes)
                record = bone.to_fabrication_record()
                self.assertEqual(record["geometry"]["status"], "unresolved")
                self.assertNotIn("verts", record["geometry"])
                report = ValidatorAgent(SkeletonField([bone])).run()
                self.assertFalse(report["export_readiness"]["ready"])
                self.assertTrue(report["missing_required_fields"])

    def test_supplied_and_unknown_mass_are_preserved_with_provenance(self):
        for mass, status in ((0.123, "supplied"), (None, "unknown")):
            with self.subTest(mass=mass):
                bone = fixture(physics={"mass_kg": mass})
                before = copy.deepcopy(bone.__dict__)
                record = bone.to_fabrication_record()
                self.assertEqual(record["physics"]["mass_kg"], mass)
                self.assertEqual(record["physics"]["mass_provenance"]["status"], status)
                self.assertEqual(bone.__dict__, before)

    def test_exported_nested_containers_cannot_mutate_the_bone(self):
        bone = fixture(geometry={"shape": "box", "origin_mm": [1, 2, 3]},
                       material={"density": 1800, "layers": [{"name": "source"}]},
                       physics={"mass_kg": 0.018, "mass_provenance": {"status": "supplied", "sources": ["source"]}})
        before = copy.deepcopy(bone.__dict__)
        record = bone.to_fabrication_record()
        record["geometry"]["origin_mm"][0] = 999
        record["material"]["layers"][0]["name"] = "export-marker"
        record["physics"]["mass_provenance"]["sources"].append("export-marker")
        record["connections"]["children"].append("export-marker")
        record["joint_interfaces"][0]["name"] = "export-marker"
        record["mount_points"][0]["name"] = "export-marker"
        self.assertEqual(bone.__dict__, before)


if __name__ == "__main__":
    unittest.main()
