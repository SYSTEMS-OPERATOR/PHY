"""Relationship declarations are observations, never mechanical qualification."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import warnings

from skeleton.base import BoneSpec
from skeleton.bones import load_field
from skeleton.field import SkeletonField
from skeleton.identity import IdentityResolver
from skeleton.relationships import build_relationship_snapshot, encode, json_safe, relationships_for_bone, relationship_markdown
from skeleton.relationship_catalog import load_catalog
from skeleton.exporters.relationship_exporter import export_relationship_snapshot
from skeleton.exporters.identity import ExportIdentity

ROOT = Path(__file__).resolve().parents[1]


def fixture(uid, name=None, references=(), **kw):
    return BoneSpec(unique_id=uid, name=name or uid, bone_type="long", location={"region": "test"},
                    articulations=[{"bone": value} for value in references], dimensions={},
                    function=[], notable_features=[], developmental_notes="synthetic", variations="", **kw)


class IdentityResolverTest(unittest.TestCase):
    def test_consumers_preserve_their_distinct_matching_policies(self):
        source, target = fixture("A", references=("pelvis",)), fixture("P", name="Pelvis")
        field = SkeletonField([source, target])
        self.assertEqual(source.entanglement_links, ["P"])
        self.assertEqual(field._connection_pairs(), [])
        identity = ExportIdentity([{"unique_id": "A", "name": "A", "connections": {"parent": "pelvis"}},
                                   {"unique_id": "P", "name": "Pelvis", "connections": {}}])
        self.assertIsNone(identity.parents["A"])
        self.assertEqual(identity.parent_references["A"]["status"], "missing")

    def test_id_precedence_ambiguity_policies_and_order(self):
        rows = [{"unique_id": "Femur", "name": "Selected"}, {"unique_id": "L", "name": "Femur"},
                {"unique_id": "R", "name": "Femur"}, {"unique_id": "P", "name": "Pelvis"}]
        for ordered in (rows, list(reversed(rows))):
            resolver = IdentityResolver(ordered)
            self.assertEqual(resolver.resolve("Femur", policy="exact")["bone_id"], "Femur")
            self.assertEqual(resolver.resolve("FEMUR", policy="case_insensitive")["candidates"], ["L", "R"])
            self.assertEqual(resolver.resolve("pelvis", policy="exact")["status"], "missing")
            self.assertEqual(resolver.resolve("pelvis", policy="case_insensitive")["bone_id"], "P")
            self.assertEqual(resolver.resolve("Pelvis", policy="id_only")["status"], "missing")
            self.assertEqual(resolver.resolve("P", policy="id_only")["bone_id"], "P")
            for invalid in (None, 7, True, [], {"id": "P"}):
                self.assertEqual(resolver.resolve(invalid, policy="exact")["status"], "invalid")
            with self.assertRaises(ValueError):
                resolver.resolve("P", policy="guess")

    def test_duplicate_and_invalid_identity_sets_are_diagnosed(self):
        resolver = IdentityResolver([{"unique_id": "A"}, {"unique_id": "A"}, {"unique_id": None}])
        self.assertTrue(resolver.fatal)
        self.assertEqual(resolver.duplicate_ids, ["A"])
        self.assertEqual(resolver.invalid_ids, ["None"])
        self.assertIsNone(resolver.resolve("A", policy="exact")["bone_id"])

    def test_optional_malformed_reference_does_not_invoke_its_equality_operator(self):
        class InvalidReference:
            def __eq__(self, other):
                raise ValueError("not a text reference")
        resolver = IdentityResolver([{"unique_id": "A", "name": "A"}])
        self.assertEqual(resolver.resolve(InvalidReference(), policy="exact", optional=True)["status"], "invalid")


class RelationshipSnapshotTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            cls.field = load_field()
        cls.graph = build_relationship_snapshot(cls.field)

    def test_real_inventory_regions_tmjs_and_hyoid(self):
        graph = self.graph
        self.assertEqual(graph["inventory"]["loaded_adult_identities"], 206)
        self.assertTrue(graph["inventory"]["complete_adult_snapshot"])
        self.assertEqual(len(graph["bones"]), 206)
        self.assertEqual(graph["counts_by_kind"]["structural_membership"], 7)
        joints = [r for r in graph["relationships"] if r["kind"] == "qualitative_articulation"]
        self.assertEqual({r["articulation_id"] for r in joints}, {"tmj_r", "tmj_l"})
        for joint in joints:
            self.assertIn("BONE_MANDIBLE", [p["bone_id"] for p in joint["participants"]])
            self.assertEqual(joint["biological_classification"], "synovial")
            self.assertEqual(joint["qualitative_motion"], ["rotation", "translation"])
            self.assertIsNone(joint["engineering_definition"])
        hyoid = relationships_for_bone(graph, "BONE_HYOID")
        self.assertEqual(len(hyoid), 1)
        self.assertEqual(hyoid[0]["kind"], "legacy_parent")
        self.assertEqual(hyoid[0]["resolution_status"], "unspecified")
        self.assertIsNone(hyoid[0]["participants"][1]["bone_id"])

    def test_legacy_hinge_and_temporal_ambiguity_are_not_rewritten(self):
        rows = relationships_for_bone(self.graph, "BONE_MANDIBLE")
        legacy = next(r for r in rows if r["kind"] == "legacy_articulation")
        self.assertEqual(legacy["original_declaration"], {"bone": "temporal", "joint_type": "hinge"})
        target = legacy["participants"][1]
        self.assertIsNone(target["bone_id"])
        self.assertEqual(target["resolution"]["status"], "missing")
        self.assertEqual(target["runtime_resolution"]["status"], "ambiguous")
        self.assertEqual(target["runtime_resolution"]["candidates"], ["BONE_TEMP_L", "BONE_TEMP_R"])
        self.assertIn(legacy, relationships_for_bone(self.graph, "BONE_TEMP_R"))

    def test_markdown_summary_is_bounded_without_dropping_json_diagnostics(self):
        before = encode(self.graph)
        report = relationship_markdown(self.graph)
        self.assertLess(len(report.splitlines()), 100)
        self.assertIn("BONE_MANDIBLE:articulations[0]", report)
        self.assertIn(f"of {len(self.graph['diagnostics']['unresolved_references'])} unresolved", report)
        self.assertEqual(encode(self.graph), before)

    def test_partial_snapshots_do_not_create_records_contacts_or_runtime_links(self):
        graph = build_relationship_snapshot(SkeletonField([fixture("BONE_MANDIBLE")]))
        self.assertEqual(graph["inventory"]["loaded_adult_identities"], 1)
        self.assertEqual(len(graph["inventory"]["missing_record_ids"]), 205)
        self.assertEqual(graph["counts_by_kind"]["operational_signal"], 0)
        for relation in graph["relationships"]:
            if relation["kind"] == "qualitative_articulation":
                self.assertEqual(relation["resolution_status"], "missing")
                self.assertEqual(sum(p["bone_id"] is not None for p in relation["participants"]), 1)
        self.assertFalse(graph["inventory"]["complete_adult_snapshot"])
        extra = build_relationship_snapshot(SkeletonField([fixture("HARDWARE_TEST")]))
        self.assertEqual(extra["inventory"]["loaded_adult_identities"], 0)
        self.assertEqual(extra["inventory"]["non_inventory_record_ids"], ["HARDWARE_TEST"])
        self.assertEqual(len(extra["bones"]), 206)

    def test_snapshot_is_pure_idempotent_and_record_order_independent(self):
        before = deepcopy(self.field.__dict__)
        first = build_relationship_snapshot(self.field)
        reordered = SkeletonField()
        reordered.bones = dict(reversed(list(self.field.bones.items())))
        second = build_relationship_snapshot(reordered)
        self.assertEqual(encode(first), encode(second))
        self.assertEqual(encode(first), encode(build_relationship_snapshot(self.field)))
        self.assertEqual(self.field.__dict__, before)

    def test_only_parents_form_parent_cycles(self):
        a, b = fixture("A"), fixture("B")
        a.connections = {"parent": None, "children": ["B"]}
        b.connections = {"parent": None, "children": ["A"]}
        field = SkeletonField([a, b])
        self.assertEqual(build_relationship_snapshot(field)["diagnostics"]["parent_cycles"], [])
        a.connections["parent"], b.connections["parent"] = "B", "A"
        self.assertEqual(build_relationship_snapshot(field)["diagnostics"]["parent_cycles"], [["A", "B", "A"]])

    def test_malformed_declarations_remain_json_safe_without_becoming_links(self):
        source, target = fixture("A"), fixture("B")
        field = SkeletonField([source, target])
        source.connections = {"parent": {"id": "B"}, "children": "B"}
        source.articulations = [None, {"bone": ["B"]}, {"bone": float("nan")}, {"bone": object()}]
        source.entanglement_links = "B"
        graph = build_relationship_snapshot(field)
        declared = [r for r in graph["relationships"] if r["relationship_id"].startswith("REL:A:")]
        self.assertTrue(all(r["resolution_status"] == "invalid" for r in declared))
        self.assertTrue(all(r["participants"][1]["bone_id"] is None for r in declared))
        self.assertEqual(next(r for r in declared if r["kind"] == "legacy_parent")["original_declaration"], {"id": "B"})
        self.assertEqual(encode(graph), encode(build_relationship_snapshot(field)))
        json.loads(encode(graph), parse_constant=lambda value: self.fail(value))
        self.assertEqual(json_safe({1: float("inf")})["invalid_type"], "mapping_keys")

    def test_runtime_domain_aliases_and_manual_links_do_not_rewrite_anatomy(self):
        a, b = fixture("A", domain_id="runtime-a"), fixture("B", domain_id="runtime-b")
        field = SkeletonField([a, b])
        a.entangle(b)
        graph = build_relationship_snapshot(field)
        links = [r for r in graph["relationships"] if r["kind"] == "operational_signal"]
        self.assertEqual(len(links), 2)
        self.assertEqual(links[0]["participants"][1]["bone_id"], "B")
        self.assertEqual(links[0]["original_declaration"], "runtime-b")
        self.assertEqual(graph["counts_by_kind"]["legacy_adjacency"], 0)
        field.propagate("runtime-a", {"voltage": 4})
        self.assertEqual((a.voltage_potential, b.voltage_potential), (0, 2))

    def test_runtime_links_follow_actual_registration_and_diagnose_stale_domain_labels(self):
        a, b = fixture("A", domain_id="runtime-a"), fixture("B", domain_id="runtime-b")
        field = SkeletonField([a, b])
        a.entangle(b)
        b.domain_id = "stale-label"
        graph = build_relationship_snapshot(field)
        link = next(r for r in graph["relationships"] if r["relationship_id"].startswith("REL:A:entanglement_links"))
        self.assertEqual(link["participants"][1]["bone_id"], "B")
        self.assertEqual(graph["diagnostics"]["runtime_binding_issues"], [{
            "bone_id": "B", "registered_domain_id": "runtime-b", "declared_domain_id": "stale-label"}])
        self.assertEqual(b.domain_id, "stale-label")

    def test_bad_record_ids_are_refused_and_fresh_inputs_change_fingerprints(self):
        field = SkeletonField([fixture("A"), fixture("B")])
        field.bones["B"].unique_id = "A"
        with self.assertRaisesRegex(ValueError, "duplicate_ids"):
            build_relationship_snapshot(field)
        field.bones["B"].unique_id = None
        with self.assertRaisesRegex(ValueError, "invalid_ids"):
            build_relationship_snapshot(field)
        field = SkeletonField([fixture("A")])
        first = build_relationship_snapshot(field)
        field.bones["A"].articulations.append({"bone": "MISSING", "joint_type": "unresolved"})
        second = build_relationship_snapshot(field)
        self.assertNotEqual(first["inputs"]["bone_declarations_sha256"], second["inputs"]["bone_declarations_sha256"])
        catalog = load_catalog()
        catalog["articulations"][0]["name"] += " (review)"
        third = build_relationship_snapshot(field, catalog=catalog)
        self.assertNotEqual(second["inputs"]["catalog_sha256"], third["inputs"]["catalog_sha256"])
        self.assertIsNone(third["inputs"]["catalog_file_sha256"])


class RelationshipPublicationTest(unittest.TestCase):
    def test_modified_qualification_or_unfingerprinted_values_cannot_replace_package(self):
        graph = build_relationship_snapshot(SkeletonField([fixture("A")]))
        with tempfile.TemporaryDirectory() as directory:
            paths = export_relationship_snapshot(graph, directory)
            before = {key: Path(path).read_bytes() for key, path in paths.items()}
            for change in ("qualification", "engineering", "hash", "inventory"):
                with self.subTest(change=change):
                    changed = deepcopy(graph)
                    if change == "qualification":
                        changed["qualification"]["fabrication_released"] = True
                    elif change == "engineering":
                        changed["relationships"][0]["engineering_definition"] = {"axis": "invented"}
                    elif change == "inventory":
                        changed["inventory"]["loaded_adult_identities"] = 206
                    else:
                        changed["relationships"][0]["original_declaration"] = "changed without a hash"
                    with self.assertRaises(ValueError):
                        export_relationship_snapshot(changed, directory)
                    self.assertEqual(before, {key: Path(path).read_bytes() for key, path in paths.items()})

    def test_repeat_exports_manifest_and_rollback(self):
        graph = build_relationship_snapshot(SkeletonField([fixture("A")]))
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            paths = export_relationship_snapshot(graph, root / "one")
            other = export_relationship_snapshot(graph, root / "two")
            before = {key: Path(path).read_bytes() for key, path in paths.items()}
            self.assertEqual(before, {key: Path(path).read_bytes() for key, path in other.items()})
            manifest = json.loads(before["manifest"])
            for item in manifest["files"].values():
                content = (Path(paths["manifest"]).parent / item["path"]).read_bytes()
                self.assertEqual(hashlib.sha256(content).hexdigest(), item["sha256"])
            import os
            real_replace = os.replace

            def fail_manifest(source, target):
                if ".stage-" in str(source) and Path(target).name == "relationship_manifest.json":
                    raise OSError("injected relationship manifest failure")
                return real_replace(source, target)

            changed = build_relationship_snapshot(SkeletonField([fixture("A", references=("MISSING",))]))
            with patch("skeleton.exporters.publication.os.replace", side_effect=fail_manifest):
                with self.assertRaises(OSError):
                    export_relationship_snapshot(changed, root / "one")
            self.assertEqual(before, {key: Path(path).read_bytes() for key, path in paths.items()})
            self.assertEqual(len(list((root / "one").iterdir())), 3)

    def test_cli_runs_outside_checkout_and_failure_preserves_package(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            command = [sys.executable, "-S", str(ROOT / "bin/export_relationship_graph.py"), "--output-root", str(root / "out")]
            run = subprocess.run(command, cwd=root, text=True, capture_output=True, timeout=30)
            self.assertEqual(run.returncode, 0, run.stderr)
            paths = json.loads(run.stdout)["paths"]
            before = {key: Path(path).read_bytes() for key, path in paths.items()}
            run = subprocess.run(command + ["--dataset", "missing-dataset"], cwd=root, text=True, capture_output=True, timeout=30)
            self.assertNotEqual(run.returncode, 0)
            self.assertFalse(json.loads(run.stdout)["published"])
            self.assertEqual(before, {key: Path(path).read_bytes() for key, path in paths.items()})


if __name__ == "__main__":
    unittest.main()
