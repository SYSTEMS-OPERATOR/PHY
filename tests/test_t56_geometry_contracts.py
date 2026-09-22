"""Synthetic software fixtures only: numbers here are NOT T-5.6 build targets."""

from copy import deepcopy
import json
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch

from PROJECTS.T56_CARBON.tools import check_convergence as gate
from PROJECTS.T56_CARBON.tools.geometry_contracts import (
    CONTRACTS, FRAME_ID, validate_record, valid_value,
)


SAMPLES = {
    "length": 10.0,
    "offset": 0,
    "point": [-1, 0, 2],
    "points": [[0, 0, 0], [1, 0, 0]],
    "transform": {"child_frame_id": "TEST_CHILD", "translation_mm": [0, 0, 0], "rpy_deg": [0, 0, 0]},
    "guide": {"type": "line", "origin_mm": [0, 0, 0], "direction_unit": [1, 0, 0], "travel_mm": [-1, 1]},
    "trajectory": [
        {"pose_id": "test_a", "position_mm": [0, 0, 0], "rpy_deg": [0, 0, 0]},
        {"pose_id": "test_b", "position_mm": [1, 0, 0], "rpy_deg": [0, 0, 1]},
    ],
    "joints": [{"joint_id": "test_joint", "origin_mm": [0, 0, 0], "axis_unit": [1, 0, 0], "range_deg": [-1, 1]}],
    "aabb": {"min_mm": [-1, -1, -1], "max_mm": [1, 1, 1]},
}


def closed_geometry(record):
    kind, _ = CONTRACTS[record["id"]]
    record.update(status="locked", value=deepcopy(SAMPLES[kind]), source="synthetic test only",
                  frame_id=FRAME_ID, tolerance={"linear_mm": 0.1})
    if kind in ("transform", "trajectory", "joints"):
        record["tolerance"]["angular_deg"] = 0.1
    return record


class GeometryContractsTest(unittest.TestCase):
    def test_every_registered_shape_has_a_valid_example(self):
        for record_id, (kind, units) in CONTRACTS.items():
            with self.subTest(record_id=record_id):
                record = closed_geometry({"id": record_id, "units": units, "required_for_simulation": True})
                self.assertEqual(validate_record(record), [])
                self.assertTrue(valid_value(kind, record["value"]))

    def test_non_measurements_fail(self):
        for kind in SAMPLES:
            for invalid in (None, "TBD", True, False, float("nan"), float("inf"), {}, []):
                with self.subTest(kind=kind, invalid=invalid):
                    self.assertFalse(valid_value(kind, invalid))

    def test_scalar_sign_and_vector_numeric_rules(self):
        self.assertFalse(valid_value("length", 0))
        self.assertFalse(valid_value("length", -1))
        self.assertTrue(valid_value("offset", -1))
        self.assertTrue(valid_value("point", [0, 0, 0]))
        for point in ([0, 0], [0, 0, 0, 0], [True, 0, 0], [float("nan"), 0, 0], [10**1000, 0, 0]):
            self.assertFalse(valid_value("point", point))

    def test_guides_need_unit_axes_and_ordered_travel(self):
        for direction in ([0, 0, 0], [2, 0, 0], [float("inf"), 0, 0]):
            value = dict(SAMPLES["guide"], direction_unit=direction)
            self.assertFalse(valid_value("guide", value))
        for limits in ([1, -1], [0, 0], [False, 1]):
            self.assertFalse(valid_value("guide", dict(SAMPLES["guide"], travel_mm=limits)))
        self.assertTrue(valid_value("guide", {"type": "polyline", "points_mm": [[0, 0, 0], [0, 1, 0]]}))
        self.assertFalse(valid_value("guide", {"type": "polyline", "points_mm": [[0, 0, 0], [0, 0, 0]]}))

    def test_trajectory_joint_and_volume_degeneracy(self):
        self.assertFalse(valid_value("trajectory", [SAMPLES["trajectory"][0]] * 2))
        self.assertFalse(valid_value("joints", SAMPLES["joints"] * 2))
        bad_joint = deepcopy(SAMPLES["joints"])
        bad_joint[0]["range_deg"] = [1, -1]
        self.assertFalse(valid_value("joints", bad_joint))
        for upper in ([-2, 1, 1], [-1, 1, 1]):
            self.assertFalse(valid_value("aabb", dict(SAMPLES["aabb"], max_mm=upper)))
        self.assertFalse(valid_value("transform", dict(SAMPLES["transform"], child_frame_id=FRAME_ID)))
        self.assertFalse(valid_value("transform", dict(SAMPLES["transform"], rotation="ambiguous")))

    def test_units_frame_source_and_tolerance_are_enforced(self):
        record = closed_geometry({"id": "GEO-SHO-006", "units": "mm", "required_for_simulation": True})
        for key, value in (("units", "cm"), ("frame_id", "world"), ("source", " "),
                           ("tolerance", -1), ("tolerance", {"linear_mm": 0}),
                           ("tolerance", {"linear_mm": True}), ("required_for_simulation", False)):
            with self.subTest(key=key, value=value):
                self.assertTrue(validate_record(dict(record, **{key: value})))


class ConvergenceRegressionTest(unittest.TestCase):
    def setUp(self):
        self.documents = {path: gate.load_json(path) for path in gate.REGISTER_PATHS.values()}
        self.profile_path = gate.PROJECT_DIR / "profiles" / "t56_domestic_frame.json"
        self.documents[self.profile_path] = gate.load_json(self.profile_path)
        self.closure_path = gate.PROFILE_PATHS["single-side"]
        self.documents[self.closure_path] = gate.load_json(self.closure_path)
        self.mission = self.documents[gate.REGISTER_PATHS["mission"]]
        self.geometry = self.documents[gate.REGISTER_PATHS["geometry"]]
        self.loads = self.documents[gate.REGISTER_PATHS["loads"]]
        self.profile = self.documents[self.profile_path]
        self.closure = self.documents[self.closure_path]

    def run_gate(self, scope="single-side"):
        with patch.object(gate, "load_json", side_effect=lambda path: deepcopy(self.documents[path])):
            return gate.audit(scope)

    def synthetic_closed_packet(self):
        for row in self.mission["required_inputs"]:
            row.update(status="locked", value=1, authority="synthetic test", evidence="synthetic test")
        for row in self.geometry["parameters"]:
            closed_geometry(row)
        self.geometry["coordinate_frame"].update(
            dimensioned_datum_status="locked", handedness="right_handed",
            datum_revision="synthetic-only", dimensioned_datum_evidence="synthetic fixture drawing")
        for row in self.loads["load_cases"]:
            row.update(status="approved", approval_evidence="synthetic test",
                       acceptance_criteria={"synthetic_limit": 1})
        for row in self.closure["article_evidence_packages"]:
            row.update(status="approved", evidence_refs=["synthetic test"])

    def test_current_state_remains_open_and_deterministic(self):
        for scope in (None, "single-side"):
            report = self.run_gate(scope)
            self.assertEqual(report, self.run_gate(scope))
            self.assertTrue(report["schema_valid"], report)
            self.assertFalse(report["scope_ready"])
            selected = self.geometry["parameters"]
            if scope == "single-side":
                ids = set(self.closure["required_geometry_parameters"])
                selected = [row for row in selected if row["id"] in ids]
            expected = sum(row["status"] != "locked" for row in selected)
            self.assertEqual(report["counts"]["open_geometry_parameters_in_scope"], expected)

    def test_synthetic_complete_packet_can_pass_both_scopes(self):
        self.synthetic_closed_packet()
        for scope in (None, "single-side"):
            report = self.run_gate(scope)
            self.assertTrue(report["scope_ready"], report)

    def test_bogus_geometry_cannot_unlock_a_complete_packet(self):
        self.synthetic_closed_packet()
        for row in self.geometry["parameters"]:
            previous = row["value"]
            for invalid in ("TBD", False, float("nan"), -1, {}):
                row["value"] = invalid
                if row["id"] == "GEO-THX-004" and invalid == -1:
                    continue  # Signed reference offset is valid.
                with self.subTest(id=row["id"], invalid=invalid):
                    self.assertFalse(self.run_gate()["scope_ready"])
            row["value"] = previous

    def test_missing_or_opted_out_geometry_fails(self):
        self.geometry["parameters"].pop()
        self.assertIn("missing required geometry parameter", str(self.run_gate()["errors"]))

    def test_consistently_wrong_span_cannot_bypass_canon(self):
        self.profile["arm_span_mm"] = 1800
        self.mission["locked_project_decisions"][1]["value"] = 1800
        self.geometry["locked_parameters"][1]["value"] = 1800
        self.assertIn("bound SOPHY canon", str(self.run_gate()["errors"]))

    def test_different_scale_is_valid_only_when_all_bindings_agree(self):
        self.profile["canon_reference"]["height_mm"] = 1800
        self.assertFalse(self.run_gate()["schema_valid"])
        self.profile.update(height_mm=1800, arm_span_mm=1800)
        for row in self.mission["locked_project_decisions"][:2] + self.geometry["locked_parameters"]:
            row["value"] = 1800
        self.assertTrue(self.run_gate()["schema_valid"])

    def test_missing_profile_and_unknown_canon_fail_closed(self):
        for reference in ({}, {"height_mm": 1676.4}, {"height_mm": 1676.4, "canon_version": "unknown"}):
            self.profile["canon_reference"] = reference
            self.assertFalse(self.run_gate()["schema_valid"])
        def missing_profile(path):
            if path == self.profile_path:
                raise FileNotFoundError("missing profile")
            return deepcopy(self.documents[path])
        with patch.object(gate, "load_json", side_effect=missing_profile):
            self.assertFalse(gate.audit()["schema_valid"])

    def test_open_or_unsubstantiated_datum_blocks_complete_packet(self):
        self.synthetic_closed_packet()
        frame = self.geometry["coordinate_frame"]
        for key, value in (("dimensioned_datum_status", "open"), ("handedness", "left_handed"),
                           ("dimensioned_datum_evidence", ""), ("datum_revision", "")):
            previous = frame[key]
            frame[key] = value
            self.assertFalse(self.run_gate()["scope_ready"])
            frame[key] = previous

    def test_case_cannot_be_omitted_even_with_recomputed_dependencies(self):
        self.closure["required_load_cases"].remove("LC-SHO-003")
        self.closure["required_mission_inputs"].remove("ME-EXT-001")
        self.assertIn("every load case assigned", str(self.run_gate()["errors"]))

    def test_malformed_registers_return_errors_not_tracebacks(self):
        for value in (None, [], [None], ["not a record"], [{"id": []}]):
            self.geometry["parameters"] = value
            self.assertFalse(self.run_gate()["schema_valid"])

    def test_malformed_profile_ids_return_errors(self):
        for value in (None, {}, [[]], [False], ["GEO-ARM-001", "GEO-ARM-001"]):
            self.closure["required_geometry_parameters"] = value
            self.assertFalse(self.run_gate()["schema_valid"])

    def test_cli_exit_codes_and_cwd_independence(self):
        script = Path(gate.__file__).resolve()
        for scope in ([], ["--profile", "single-side"]):
            for strict, expected in (([], 0), (["--strict"], 2)):
                result = subprocess.run([sys.executable, str(script), "--compact", *scope, *strict],
                                        cwd=script.parent, text=True, capture_output=True)
                self.assertEqual(result.returncode, expected, result.stderr)
                self.assertFalse(json.loads(result.stdout)["scope_ready"])


if __name__ == "__main__":
    unittest.main()
