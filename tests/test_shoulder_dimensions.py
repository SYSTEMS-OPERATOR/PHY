"""Synthetic geometry only: these numbers are never fabrication inputs."""
import copy
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT / "PROJECTS/T56_CARBON/tools/shoulder_dimensions.py"
spec = importlib.util.spec_from_file_location("shoulder_dimensions", TOOL)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def synthetic():
    values = ([0, 0, 0], [300, 400, 0], 30, 20, 10, 15)
    return {"model": module.MODEL, "frame_id": "FRAME_T56_THORAX",
            "pose_id": "synthetic_only", "datum_drawing": "SYNTHETIC SOFTWARE FIXTURE",
            "records": {name: {"value": value, "units": "mm", "tolerance_mm": 0.1,
                                "evidence": "SYNTHETIC SOFTWARE FIXTURE"}
                        for name, value in zip(module.FIELDS, values)}}


class ShoulderDimensionTests(unittest.TestCase):
    def test_vector_length_and_cut_offsets(self):
        result = module.review(synthetic())
        self.assertEqual(result["status"], "dimensional_review_only")
        self.assertEqual(result["dimensions_mm"], {
            "effective_length": 500, "seat_gap": 450, "stock_cut_length": 475})
        self.assertEqual(result["points_mm"]["cut_S"], [12, 16, 0])
        self.assertEqual(result["points_mm"]["cut_E"], [297, 396, 0])
        self.assertAlmostEqual(result["worst_case_bounds_mm"]["stock_cut_length"], 0.6)
        self.assertFalse(result["fabrication_ready"])
        self.assertFalse(result["canon_adopted"])

    def test_translation_does_not_change_lengths(self):
        data = synthetic()
        for key in module.POINTS:
            data["records"][key]["value"] = [v + t for v, t in zip(
                data["records"][key]["value"], [-800, 20, 75])]
        self.assertEqual(module.review(data)["dimensions_mm"], module.review(synthetic())["dimensions_mm"])

    def test_rejects_wrong_units_nonfinite_boolean_and_negative(self):
        for value in (True, float("nan"), float("inf"), -2, "20", 10**400):
            with self.subTest(value=str(value)[:20]):
                data = synthetic()
                data["records"]["shoulder_offset"]["value"] = value
                self.assertTrue(module.review(data)["errors"])
        data = synthetic()
        data["records"]["shoulder_output"]["units"] = "cm"
        self.assertTrue(module.review(data)["errors"])

    def test_requires_datum_pose_and_evidence(self):
        for name in ("pose_id", "datum_drawing"):
            data = synthetic(); data[name] = None
            self.assertTrue(module.review(data)["blockers"])
        data = synthetic(); data["records"]["shoulder_output"]["evidence"] = None
        self.assertTrue(module.review(data)["blockers"])

    def test_rejects_wrong_frame_model_and_unknown_records(self):
        for key, value in (("frame_id", "SOPHY_CANON"), ("model", "bent_member")):
            data = synthetic(); data[key] = value
            self.assertTrue(module.review(data)["errors"])
        data = synthetic(); data["records"]["acromion"] = {}
        self.assertTrue(module.review(data)["errors"])

    def test_requires_positive_tolerance_and_nonoverlapping_seats(self):
        for value in (0, -1, True):
            data = synthetic(); data["records"]["shoulder_offset"]["tolerance_mm"] = value
            self.assertTrue(module.review(data)["errors"])
        data = synthetic(); data["records"]["shoulder_offset"]["value"] = 479.8
        self.assertTrue(module.review(data)["errors"])
        data = synthetic(); data["records"]["dummy_elbow_station"]["value"] = [0, 0, 0]
        self.assertTrue(module.review(data)["errors"])

    def test_current_a0_input_resolves_without_mutation_or_release(self):
        path = ROOT / "PROJECTS/T56_CARBON/requirements/shoulder_member_inputs.json"
        data = json.loads(path.read_text()); original = copy.deepcopy(data)
        result = module.review(data)
        self.assertFalse(result["errors"])
        self.assertFalse(result["blockers"])
        self.assertEqual(result["status"], "dimensional_review_only")
        self.assertFalse(result["fabrication_ready"])
        self.assertFalse(result["canon_adopted"])
        self.assertEqual(data, original)
        self.assertIn("NOT FOR FABRICATION", module.svg(result))

    def test_cli_determinism_and_stale_drawing_removal(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp); source = directory / "input.json"
            output = directory / "output"
            source.write_text(json.dumps(synthetic()))
            command = [sys.executable, "-S", str(TOOL), "--input", str(source), "--output-dir", str(output)]
            self.assertEqual(subprocess.run(command, capture_output=True).returncode, 0)
            first = {p.name: p.read_bytes() for p in output.iterdir()}
            self.assertEqual(subprocess.run(command, capture_output=True).returncode, 0)
            self.assertEqual(first, {p.name: p.read_bytes() for p in output.iterdir()})
            data = synthetic(); data["records"]["shoulder_output"]["value"] = None
            source.write_text(json.dumps(data))
            self.assertEqual(subprocess.run(command, capture_output=True).returncode, 2)
            self.assertFalse((output / "shoulder_member_review.svg").exists())

    def test_cli_rejects_nonfinite_json_without_traceback(self):
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp) / "input.json"
            for raw in ('{"value": NaN}', '{"value": 1e309}'):
                source.write_text(raw)
                run = subprocess.run([sys.executable, str(TOOL), "--input", str(source),
                                      "--output-dir", str(Path(tmp) / "out")], capture_output=True)
                self.assertEqual(run.returncode, 1)
                self.assertNotIn(b"Traceback", run.stderr)


if __name__ == "__main__":
    unittest.main()
