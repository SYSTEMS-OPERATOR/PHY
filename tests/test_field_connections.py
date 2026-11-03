"""Rendering and signal links must preserve identity without guessing a side."""
import unittest

from skeleton.base import BoneSpec
from skeleton.field import SkeletonField
from skeleton.relationships import build_relationship_snapshot, encode
from skeleton.validation.validator_agent import ValidatorAgent


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


class ArticulationInputTest(unittest.TestCase):
    invalid_targets = (None, False, True, 7, 1.5, [], {}, ["FEMUR_L"], {"id": "FEMUR_L"})

    def test_nontext_targets_do_not_crash_or_prevent_valid_signal_links(self):
        for reference in self.invalid_targets:
            with self.subTest(reference=reference):
                target = bone("FEMUR_L", "Femur")
                decoy = bone("7", "7")
                source = bone("TIBIA_L", "Tibia", reference, "FEMUR_L")
                field = SkeletonField([target, decoy, source])
                field.build_entanglement()
                field.propagate(source.domain_id, {"voltage": 4.0})

                self.assertEqual(source.entanglement_links, [target.domain_id])
                self.assertEqual(target.entanglement_links, [source.domain_id])
                self.assertEqual(decoy.entanglement_links, [])
                self.assertEqual((source.voltage_potential, target.voltage_potential,
                                  decoy.voltage_potential), (0.0, 2.0, 0.0))
                self.assertEqual(source.articulations, [{"bone": reference}, {"bone": "FEMUR_L"}])

    def test_nontext_targets_do_not_crash_render_connection_lookup(self):
        for reference in self.invalid_targets:
            with self.subTest(reference=reference):
                # Registration alone isolates rendering from automatic wiring.
                field = SkeletonField()
                field.register(bone("FEMUR_L", "Femur"))
                field.register(bone("7", "7"))
                field.register(bone("TIBIA_L", "Tibia", reference, "FEMUR_L"))

                self.assertEqual(field._connection_pairs(), [("TIBIA_L", "FEMUR_L")])

    def test_invalid_declared_connections_remain_visible_to_validation(self):
        for reference in (True, 7, ["FEMUR_L"], {"id": "FEMUR_L"}):
            with self.subTest(reference=reference):
                target = bone("FEMUR_L", "Femur")
                source = bone("TIBIA_L", "Tibia", reference, "FEMUR_L")
                field = SkeletonField([source, target])

                report = ValidatorAgent(field).run()

                self.assertFalse(report["summary"]["pass"])
                self.assertTrue(report["invalid_parent_child_references"])
                self.assertIn("invalid_parent_child_references", report["export_readiness"]["issues"])
                self.assertEqual(source.connections["children"], [reference, "FEMUR_L"])


class ArticulationShapeTest(unittest.TestCase):
    invalid_entries = (None, False, 7, 1.5, "DECOY", [], ["DECOY"])
    invalid_containers = (None, False, 7, 1.5, "DECOY", {"bone": "DECOY"},
                          ({"bone": "DECOY"},))

    @staticmethod
    def source(articulations):
        # Explicit canonical connections allow the original malformed legacy
        # declaration to reach field consumers without deriving adjacency.
        return BoneSpec(
            unique_id="SOURCE", name="Source", bone_type="long", location={"region": "test"},
            articulations=articulations, connections={"parent": None, "children": []},
            dimensions={}, function=[], notable_features=[],
            developmental_notes="synthetic articulation-shape fixture", variations="",
        )

    def test_invalid_entries_do_not_block_neighboring_signal_delivery_or_diagnostics(self):
        for entry in self.invalid_entries:
            with self.subTest(entry=entry):
                original = [entry, {"bone": "TARGET"}]
                source, target, decoy = self.source(original), bone("TARGET", "Target"), bone("DECOY", "Decoy")
                field = SkeletonField([source, target, decoy])
                field.build_entanglement()
                field.propagate("SOURCE", {"voltage": 4.0})
                self.assertEqual(source.entanglement_links, ["TARGET"])
                self.assertEqual(target.entanglement_links, ["SOURCE"])
                self.assertEqual(decoy.entanglement_links, [])
                self.assertEqual(field.summary(), {"SOURCE": 0.0, "TARGET": 2.0, "DECOY": 0.0})
                self.assertIs(source.articulations, original)
                self.assertEqual(original, [entry, {"bone": "TARGET"}])

                graph = build_relationship_snapshot(field)
                row = next(r for r in graph["relationships"]
                           if r["relationship_id"] == "REL:SOURCE:articulations[0]")
                self.assertEqual(row["resolution_status"], "invalid")
                self.assertEqual(row["original_declaration"], entry)
                self.assertIsNone(row["participants"][1]["bone_id"])
                self.assertIsNone(row["participants"][1]["runtime_resolution"]["bone_id"])
                self.assertEqual(encode(graph), encode(build_relationship_snapshot(field)))

    def test_invalid_entries_do_not_crash_rendering_or_become_edges(self):
        for entry in self.invalid_entries:
            with self.subTest(entry=entry):
                source = self.source([entry, {"bone": "TARGET"}])
                # Isolate rendering from automatic wiring to detect either
                # consumer regressing independently.
                field = SkeletonField()
                for item in (source, bone("TARGET", "Target"), bone("DECOY", "Decoy")):
                    field.register(item)
                self.assertEqual(field._connection_pairs(), [("SOURCE", "TARGET")])
                self.assertEqual(source.entanglement_links, [])

    def test_invalid_containers_are_diagnosed_without_interpreting_their_contents(self):
        for container in self.invalid_containers:
            with self.subTest(container=container):
                source = self.source(container)
                neighbor = bone("NEIGHBOR", "Neighbor", "TARGET")
                field = SkeletonField([source, neighbor, bone("TARGET", "Target"), bone("DECOY", "Decoy")])
                field.propagate("NEIGHBOR", {"voltage": 4.0})
                self.assertEqual(field.summary(), {"SOURCE": 0.0, "NEIGHBOR": 0.0, "TARGET": 2.0, "DECOY": 0.0})
                self.assertEqual(source.entanglement_links, [])
                self.assertEqual(field._connection_pairs(), [("NEIGHBOR", "TARGET")])
                self.assertIs(source.articulations, container)
                graph = build_relationship_snapshot(field)
                row = next(r for r in graph["relationships"]
                           if r["relationship_id"] == "REL:SOURCE:articulations")
                self.assertEqual(row["resolution_status"], "invalid")
                self.assertIsNone(row["participants"][1]["bone_id"])
                self.assertEqual(row["original_declaration"], container if not isinstance(container, tuple)
                                 else {"invalid_type": "builtins.tuple"})


if __name__ == "__main__":
    unittest.main()
