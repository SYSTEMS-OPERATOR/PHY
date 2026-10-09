"""Export-contract regressions. Synthetic fixtures are software evidence only."""
from contextlib import redirect_stdout
from copy import deepcopy
import hashlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from skeleton.base import BoneSpec
from skeleton.exporters import ExportError, ExporterAgent
from skeleton.exporters.cli import main
from skeleton.exporters.exporter_agent import json_bytes
from skeleton.field import SkeletonField


REPO = Path(__file__).resolve().parents[1]


def fixture(uid="EXPORT_FIXTURE", name=None, parent=None, children=None, **changes):
    values = dict(
        unique_id=uid, name=name or uid, bone_type="long", location={"region": "test"},
        articulations=[], function=[], notable_features=[], developmental_notes="synthetic",
        variations="", dimensions={"length_mm": 100, "width_mm": 10, "thickness_mm": 10},
        geometry={"shape": "box"}, material={"name": "synthetic", "density": 1800},
        physics={"mass_kg": 0.018}, connections={"parent": parent, "children": children or []},
        joint_interfaces=[{"name": "shared_joint_label"}], mount_points=[{"name": "test_mount"}],
        manufacturing_notes=["Synthetic fixture, not a released part"], references=["test-evidence"],
    )
    values.update(changes)
    return BoneSpec(**values)


def read_json(path):
    def reject(value):
        raise AssertionError(f"Nonfinite JSON constant: {value}")
    return json.loads(Path(path).read_text(encoding="utf-8"), parse_constant=reject)


def export(field, root, mode="review"):
    return ExporterAgent(field).export_all(root / "dist", root / "reports", root / "exports", mode=mode)


def package_bytes(paths):
    return {key: Path(path).read_bytes() for key, path in paths.items()}


class ExportContractTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def test_validated_package_has_verified_content_and_input_hashes(self):
        bones = [fixture("B", parent="A"), fixture("A", children=["B"])]
        dataset = {"A": {}, "B": {}, "source": "synthetic"}
        for bone in bones:
            bone.dataset = deepcopy(dataset)
            bone.dataset_key = bone.unique_id
        records = [bone.to_fabrication_record() for bone in reversed(bones)]
        paths = export(SkeletonField(bones), self.root, "validated")
        self.assertEqual(len(paths), 10)
        manifest = read_json(paths["manifest"])
        self.assertTrue(manifest["validated_ready"])
        self.assertFalse(manifest["fabrication_released"])
        self.assertEqual(manifest["mode"], "validated")
        self.assertEqual(manifest["record_count"], 2)
        self.assertEqual(set(manifest["files"]), set(paths) - {"manifest"})
        for key, entry in manifest["files"].items():
            target = Path(paths["manifest"]).parent / entry["path"]
            self.assertEqual(target.resolve(), Path(paths[key]).resolve())
            self.assertEqual(hashlib.sha256(target.read_bytes()).hexdigest(), entry["sha256"])
            read_json(target)
        source = manifest["source"]
        self.assertEqual(source["source_records_sha256"], hashlib.sha256(json_bytes(records)).hexdigest())
        self.assertEqual(source["schema"]["sha256"],
                         hashlib.sha256((REPO / source["schema"]["path"]).read_bytes()).hexdigest())
        self.assertEqual(source["datasets"], [{"bone_ids": ["A", "B"],
                                              "sha256": hashlib.sha256(json_bytes(dataset)).hexdigest()}])
        self.assertEqual(source["revisions"], {"A": "1.0.0", "B": "1.0.0"})

    def test_bilateral_names_never_replace_machine_identity(self):
        field = SkeletonField([
            fixture("PELVIS", name="Pelvis", children=["FEMUR_L", "FEMUR_R"]),
            fixture("FEMUR_R", name="Femur", parent="PELVIS"),
            fixture("FEMUR_L", name="Femur", parent="Pelvis"),
        ])
        paths = export(field, self.root, "validated")
        records = {row["unique_id"]: row for row in read_json(paths["canonical_json"])}
        self.assertEqual(records["FEMUR_L"]["source_connections"]["parent"], "Pelvis")
        self.assertEqual(records["FEMUR_L"]["connections"]["parent"], "PELVIS")
        self.assertEqual(records["FEMUR_L"]["connection_resolution"]["parent"]["status"],
                         "unique_display_name")
        frames = read_json(paths["tf_tree"])
        self.assertEqual({row["frame_id"] for row in frames}, set(field.bones))
        self.assertEqual(next(row for row in frames if row["frame_id"] == "PELVIS")["children"],
                         ["FEMUR_L", "FEMUR_R"])
        robot = read_json(paths["urdf_like"])
        self.assertEqual({row["name"] for row in robot["links"]}, set(field.bones))
        self.assertEqual(len({row["name"] for row in robot["joints"]}), 3)
        self.assertEqual({row["display_name"] for row in robot["joints"]}, {"shared_joint_label"})
        for key in ("material_table", "joint_table"):
            self.assertEqual({row["bone"] for row in read_json(paths[key])}, set(field.bones))
        self.assertEqual({row["part_number"] for row in read_json(paths["bom"])["items"]}, set(field.bones))

    def test_ambiguous_display_reference_is_reported_without_a_side_guess(self):
        field = SkeletonField([fixture("FEMUR_L", name="Femur"), fixture("FEMUR_R", name="Femur"),
                               fixture("TIBIA_L", parent="Femur")])
        paths = export(field, self.root)
        report = read_json(paths["export_report"])
        self.assertTrue(report["record_validation_pass"])
        self.assertFalse(report["validated_ready"])
        issue = report["identity_validation"]["unresolved_references"][0]
        self.assertEqual(issue["bone_id"], "TIBIA_L")
        self.assertEqual(issue["status"], "ambiguous")
        self.assertEqual(issue["candidates"], ["FEMUR_L", "FEMUR_R"])
        tibia = next(row for row in read_json(paths["canonical_json"]) if row["unique_id"] == "TIBIA_L")
        self.assertIsNone(tibia["connections"]["parent"])
        self.assertEqual(tibia["source_connections"]["parent"], "Femur")
        before = package_bytes(paths)
        with self.assertRaises(ExportError) as caught:
            export(field, self.root, "validated")
        self.assertEqual(caught.exception.code, "validation_failed")
        self.assertEqual(before, package_bytes(paths))

    def test_exact_id_wins_over_duplicate_display_names(self):
        field = SkeletonField([fixture("Femur", name="Other"), fixture("L", name="Femur"),
                               fixture("R", name="Femur"), fixture("CHILD", parent="Femur")])
        paths = export(field, self.root, "validated")
        child = next(row for row in read_json(paths["tf_tree"]) if row["frame_id"] == "CHILD")
        self.assertEqual(child["parent"], "Femur")
        self.assertEqual(child["parent_reference"]["status"], "id")

    def test_missing_misspelled_and_invalid_references_remain_explicit(self):
        field = SkeletonField([fixture("ROOT", name="Root"),
                               fixture("CHILD", parent="root", children=["MISSING", None, {"id": "ROOT"}])])
        paths = export(field, self.root)
        issues = read_json(paths["export_report"])["identity_validation"]["unresolved_references"]
        self.assertEqual([row["status"] for row in issues], ["missing", "missing", "invalid", "invalid"])
        child = next(row for row in read_json(paths["canonical_json"]) if row["unique_id"] == "CHILD")
        self.assertIsNone(child["connections"]["parent"])
        self.assertEqual(child["connections"]["children"], [])
        self.assertEqual(child["source_connections"], field.bones["CHILD"].connections)

    def test_malformed_children_container_is_not_promoted_to_topology(self):
        bone = fixture(connections={"parent": None, "children": {"child": "EXPORT_FIXTURE"}})
        paths = export(SkeletonField([bone]), self.root)
        report = read_json(paths["export_report"])
        self.assertFalse(report["validated_ready"])
        self.assertEqual(report["identity_validation"]["unresolved_references"][0]["status"], "invalid")
        self.assertEqual(read_json(paths["canonical_json"])[0]["connections"]["children"], [])

    def test_declared_parent_cycles_block_validated_packages(self):
        for parents, expected in (({"A": "B", "B": "A"}, [["A", "B", "A"]]),
                                  ({"A": "A"}, [["A", "A"]])):
            with self.subTest(parents=parents):
                field = SkeletonField([fixture(uid, parent=parent) for uid, parent in parents.items()])
                paths = export(field, self.root)
                report = read_json(paths["export_report"])
                self.assertEqual(report["identity_validation"]["parent_cycles"], expected)
                self.assertFalse(report["validated_ready"])
                with self.assertRaises(ExportError) as caught:
                    export(field, self.root, "validated")
                self.assertEqual(caught.exception.code, "validation_failed")

    def test_anatomical_adjacency_is_not_an_additional_parent_graph(self):
        field = SkeletonField([fixture("A", children=["B"]), fixture("B", children=["A"])])
        paths = export(field, self.root, "validated")
        self.assertTrue(read_json(paths["export_report"])["validated_ready"])
        self.assertTrue(all(row["parent"] is None and row["children"] == []
                            for row in read_json(paths["tf_tree"])))
        self.assertEqual(read_json(paths["canonical_json"])[0]["connections"]["children"], ["B"])

    def test_source_geometry_origin_does_not_invent_a_scapula_pose_or_joint(self):
        bone = fixture("SCAPULA_L", name="Scapula", geometry={"shape": "box", "origin_mm": [1, 2, 3]})
        paths = export(SkeletonField([bone]), self.root, "validated")
        frame = read_json(paths["tf_tree"])[0]
        self.assertIsNone(frame["origin_mm"])
        self.assertIsNone(frame["rotation_rpy_rad"])
        self.assertEqual(frame["transform_status"], "unresolved")
        robot = read_json(paths["urdf_like"])
        self.assertFalse(robot["mechanical_transforms_validated"])
        self.assertEqual(robot["role"], "review_inventory")
        self.assertIsNone(robot["joints"][0]["type"])
        self.assertEqual(robot["joints"][0]["definition_status"], "unqualified")
        self.assertEqual(robot["links"][0]["geometry"]["origin_mm"], [1, 2, 3])

    def test_unknown_measurements_and_mass_remain_null_in_review(self):
        bone = fixture(dimensions={"length_cm": None, "width_m": None, "thickness_cm": 1},
                       physics={"mass_kg": None}, references=[])
        paths = export(SkeletonField([bone]), self.root)
        record = read_json(paths["canonical_json"])[0]
        self.assertEqual(record["dimensions"], {"length_mm": None, "width_mm": None, "thickness_mm": 10})
        self.assertIsNone(record["physics"]["mass_kg"])
        self.assertEqual(record["geometry"]["status"], "unresolved")
        self.assertFalse(read_json(paths["manifest"])["validated_ready"])
        self.assertEqual(read_json(paths["reference_audit"])["missing_references"], [bone.unique_id])
        with self.assertRaises(ExportError):
            export(SkeletonField([bone]), self.root, "validated")

    def test_nonfinite_record_and_dataset_values_cannot_replace_a_package(self):
        paths = export(SkeletonField([fixture()]), self.root, "validated")
        before = package_bytes(paths)
        cases = [fixture(physics={"mass_kg": 0.018, "nested": [float("nan")]}),
                 fixture(material={"name": "synthetic", "density": float("inf")}),
                 fixture(physics={"mass_kg": float("-inf")}),
                 fixture(physics={"mass_kg": 0.018, "nested": 10**400}),
                 fixture(dimensions={"length_mm": 1e200, "width_mm": 1e200, "thickness_mm": 1e200}, physics={})]
        unused_dataset = fixture()
        unused_dataset.dataset = {unused_dataset.unique_id: {}, "unused": float("nan")}
        unused_dataset.dataset_key = unused_dataset.unique_id
        cases.append(unused_dataset)
        for index, bone in enumerate(cases):
            for mode in ("review", "validated"):
                with self.subTest(case=index, mode=mode), self.assertRaises(ExportError) as caught:
                    export(SkeletonField([bone]), self.root, mode)
                self.assertIn(caught.exception.code, ("serialization_failed", "validation_failed"))
                json.dumps(caught.exception.report, allow_nan=False)
                self.assertEqual(before, package_bytes(paths))

    def test_one_conversion_failure_never_publishes_a_subset(self):
        field = SkeletonField([fixture("VALID"), fixture("CONFLICT", dimensions={"length_mm": 100, "length_cm": 11})])
        with self.assertRaises(ExportError) as caught:
            export(field, self.root)
        self.assertEqual(caught.exception.code, "record_conversion_failed")
        self.assertEqual(caught.exception.report["issues"][0]["bone_id"], "CONFLICT")
        self.assertEqual(list(self.root.rglob("*.json")), [])

    def test_mutated_invalid_and_duplicate_ids_block_every_mode(self):
        for invalid_id in ("A", "", None):
            field = SkeletonField([fixture("A"), fixture("B")])
            field.bones["B"].unique_id = invalid_id
            for mode in ("review", "validated"):
                with self.subTest(uid=invalid_id, mode=mode), self.assertRaises(ExportError) as caught:
                    export(field, self.root, mode)
                self.assertEqual(caught.exception.code, "invalid_identity")
                json.dumps(caught.exception.report, allow_nan=False)
        self.assertEqual(list(self.root.rglob("*.json")), [])

    def test_snapshot_and_bytes_are_independent_of_insertion_order(self):
        bones = [fixture("B", parent="A"), fixture("A", children=["B"])]
        before = [deepcopy(bone.__dict__) for bone in bones]
        first = export(SkeletonField(bones), self.root / "first")
        second = export(SkeletonField(list(reversed(bones))), self.root / "second")
        self.assertEqual(package_bytes(first), package_bytes(second))
        self.assertEqual([bone.__dict__ for bone in bones], before)


class PublicationTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def test_staging_failure_preserves_the_previous_package(self):
        paths = export(SkeletonField([fixture()]), self.root, "validated")
        before = package_bytes(paths)
        with patch("skeleton.exporters.publication.os.fsync", side_effect=OSError("test disk failure")):
            with self.assertRaises(ExportError) as caught:
                export(SkeletonField([fixture(dimensions={"length_mm": 200})]), self.root)
        self.assertEqual(caught.exception.code, "publication_failed")
        self.assertTrue(caught.exception.report["previous_package_restored"])
        self.assertEqual(before, package_bytes(paths))
        self.assertFalse(any(".stage-" in p.name or ".backup-" in p.name for p in self.root.rglob("*")))

    def test_promotion_failure_restores_all_existing_files_including_manifest(self):
        paths = export(SkeletonField([fixture()]), self.root, "validated")
        before = package_bytes(paths)
        replace = os.replace

        def fail_manifest(source, target):
            if ".stage-" in Path(source).name and Path(target).name == "export_manifest.json":
                raise OSError("test manifest failure after all artifacts")
            return replace(source, target)

        with patch("skeleton.exporters.publication.os.replace", side_effect=fail_manifest):
            with self.assertRaises(ExportError) as caught:
                export(SkeletonField([fixture(dimensions={"length_mm": 200})]), self.root)
        self.assertTrue(caught.exception.report["previous_package_restored"])
        self.assertEqual(before, package_bytes(paths))
        self.assertFalse(any(".stage-" in p.name or ".backup-" in p.name for p in self.root.rglob("*")))

    def test_failed_first_publication_removes_partial_package_files(self):
        replace = os.replace

        def fail_tf(source, target):
            if ".stage-" in Path(source).name and Path(target).name == "ros_tf_tree.json":
                raise OSError("test partial publication failure")
            return replace(source, target)

        with patch("skeleton.exporters.publication.os.replace", side_effect=fail_tf):
            with self.assertRaises(ExportError) as caught:
                export(SkeletonField([fixture()]), self.root)
        self.assertTrue(caught.exception.report["previous_package_restored"])
        self.assertEqual(list(self.root.rglob("*.json")), [])

    def test_rollback_failure_reports_and_retains_recoverable_prior_bytes(self):
        paths = export(SkeletonField([fixture()]), self.root, "validated")
        before = package_bytes(paths)
        replace = os.replace

        def fail_publication_and_one_restore(source, target):
            if ".stage-" in Path(source).name and Path(target).name == "skeleton_urdf_like.json":
                raise OSError("test promotion failure")
            if ".backup-" in Path(source).name and Path(target).name == "skeleton_canonical.json":
                raise OSError("test restore failure")
            return replace(source, target)

        with patch("skeleton.exporters.publication.os.replace", side_effect=fail_publication_and_one_restore):
            with self.assertRaises(ExportError) as caught:
                export(SkeletonField([fixture(dimensions={"length_mm": 200})]), self.root)
        report = caught.exception.report
        self.assertFalse(report["previous_package_restored"])
        self.assertEqual(len(report["rollback_errors"]), 1)
        self.assertEqual(len(report["recovery_files"]), 1)
        recovery = report["recovery_files"][0]
        self.assertEqual(Path(recovery["backup"]).read_bytes(), before["canonical_json"])
        self.assertEqual(Path(paths["manifest"]).read_bytes(), before["manifest"])


class CliContractTest(unittest.TestCase):
    def test_both_entrypoints_block_by_default_and_explicitly_publish_review(self):
        with tempfile.TemporaryDirectory() as directory:
            outside = Path(directory)
            for script in ("bin/export_fabrication.py", "assemble_skeleton.py"):
                root = outside / Path(script).stem
                command = [sys.executable, str(REPO / script), "--output-root", str(root)]
                env = {**os.environ, "PYTHONPATH": str(REPO)}
                with self.subTest(script=script):
                    failed = subprocess.run(command, cwd=outside, env=env, text=True, capture_output=True, check=False)
                    self.assertEqual(failed.returncode, 1, failed.stderr)
                    result = json.loads(failed.stdout)
                    self.assertFalse(result["published"])
                    self.assertNotIn("paths", result)
                    report = read_json(result["diagnostic_report"])
                    self.assertEqual(report["code"], "validation_failed")
                    self.assertEqual(report["record_validation"]["summary"]["total_bones"], 193)
                    self.assertFalse((root / "dist" / "export_manifest.json").exists())
                    review = subprocess.run(command + ["--mode", "review"], cwd=outside, env=env,
                                            text=True, capture_output=True, check=False)
                    self.assertEqual(review.returncode, 0, review.stderr)
                    result = json.loads(review.stdout)
                    self.assertTrue(result["published"])
                    self.assertFalse(result["validated_ready"])
                    self.assertFalse(result["fabrication_released"])
                    before = package_bytes(result["paths"])
                    manifest = read_json(result["paths"]["manifest"])
                    self.assertEqual(manifest["mode"], "review")
                    self.assertEqual(len(manifest["files"]), 9)
                    self.assertNotIn("export_failure.json", str(manifest["files"]))
                    blocked = subprocess.run(command, cwd=outside, env=env, text=True, capture_output=True, check=False)
                    self.assertEqual(blocked.returncode, 1, blocked.stderr)
                    self.assertEqual(before, package_bytes(result["paths"]))

    def test_positive_control_cli_publishes_without_suppressing_the_gate(self):
        with tempfile.TemporaryDirectory() as directory, patch("skeleton.exporters.cli.load_field",
                                                              return_value=SkeletonField([fixture()])):
            output = io.StringIO()
            with redirect_stdout(output):
                status = main(["--output-root", directory])
            self.assertEqual(status, 0)
            result = json.loads(output.getvalue())
            self.assertEqual(result["mode"], "validated")
            self.assertTrue(result["validated_ready"])
            self.assertFalse(result["fabrication_released"])

    def test_bad_dataset_has_a_strict_separate_failure_diagnostic(self):
        with tempfile.TemporaryDirectory() as directory:
            output = io.StringIO()
            with redirect_stdout(output):
                status = main(["--dataset", "NONEXISTENT_EXPORT_TEST", "--output-root", directory])
            self.assertEqual(status, 1)
            result = json.loads(output.getvalue())
            self.assertEqual(result["code"], "input_failed")
            self.assertFalse(read_json(result["diagnostic_report"])["published"])
            self.assertEqual(list(Path(directory).rglob("*.json")), [Path(result["diagnostic_report"])])


if __name__ == "__main__":
    unittest.main()
