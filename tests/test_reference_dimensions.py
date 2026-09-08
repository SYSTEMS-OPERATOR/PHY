from copy import deepcopy
from hashlib import sha256
import json
from pathlib import Path
import tempfile
import subprocess
import sys
import unittest

from PROJECTS.T56_CARBON.tools import reference_dimensions as tool


class ReferenceDimensionsTest(unittest.TestCase):
    def setUp(self):
        self.catalog = tool.load(tool.REFERENCES / "dimensional_sources.json")
        self.snapshot = tool.load(tool.REFERENCES / "ansur_female_dimensions.json")
        self.policy = tool.load(tool.REFERENCES / "dimensional_selection.json")
        self.profile = tool.load(tool.PROJECT / "profiles/t56_domestic_frame.json")
        self.legacy = tool.load(tool.ROOT / "PROJECTS/REDWOOD/profiles/adult_female_21_28.json")

    def compile(self):
        return tool.compile_specs(self.snapshot, self.catalog, self.policy, self.profile, self.legacy)

    def test_actual_reference_reproduces_committed_specs(self):
        expected = tool.load(tool.PROJECT / "reports/reference_dimension_specs.json")
        self.assertEqual(self.compile(), expected)
        self.assertEqual(self.compile()["selection"]["cohort_n"], 80)

    def test_snapshot_tampering_and_wrong_units_rejected(self):
        self.snapshot["rows"][0][2] += 1
        with self.assertRaisesRegex(ValueError, "SHA-256"):
            self.compile()
        self.snapshot["units"] = "cm"
        self.catalog["ansur"]["projected_snapshot_sha256"] = tool.digest(self.snapshot)
        with self.assertRaisesRegex(ValueError, "units"):
            self.compile()

    def test_selection_uses_measured_height_span_and_age(self):
        rows = [{"age_years": 22, "stature": 1000, "span": 1000},
                {"age_years": 29, "stature": 1000, "span": 1000},
                {"age_years": 22, "stature": 1100, "span": 1000},
                {"age_years": 22, "stature": 1000, "span": 1100}]
        self.assertEqual(tool.select(rows, 1000, [21, 28], 25, 25), rows[:1])

    def test_insufficient_sample_does_not_widen(self):
        self.policy.update(stature_band_mm=10, span_band_mm=10)
        with self.assertRaisesRegex(ValueError, "12 < 20"):
            self.compile()

    def test_invalid_policies_and_canon_drift_rejected(self):
        for field, value in (("age_years", [28, 21]), ("stature_band_mm", 0),
                             ("span_band_mm", float("nan")), ("minimum_cohort_size", True)):
            with self.subTest(field=field):
                policy = deepcopy(self.policy)
                policy[field] = value
                with self.assertRaises(ValueError):
                    tool.compile_specs(self.snapshot, self.catalog, policy, self.profile, self.legacy)
        self.profile["arm_span_mm"] += 1
        with self.assertRaisesRegex(ValueError, "canon"):
            self.compile()

    def test_reference_does_not_become_a_fabrication_value(self):
        report = self.compile()
        self.assertFalse(report["fabrication_ready"])
        for row in report["dimensions"]:
            self.assertIsNone(row["fabrication_value_mm"])
            self.assertFalse(row["canon_adopted"])
        rows = {r["measurement"]: r for r in report["dimensions"]}
        self.assertIn("not_bone", rows["acromionradialelength"]["mapping"])
        self.assertEqual(rows["handlength"]["reference_median_mm"], 183)
        self.assertAlmostEqual(rows["handlength"]["legacy_at_target_height_mm"], 177.8)
        self.assertIn("ulna_length_mm", report["unresolved_targets"])

    def test_historical_conflict_preserved(self):
        history = {r["measurement"]: r for r in self.compile()["historical_comparison"]}
        self.assertAlmostEqual(history["footlength"]["historical_mm"], 279.4)
        self.assertEqual(history["footlength"]["empirical_median_mm"], 250)
        self.assertFalse(history["footlength"]["canon_adopted"])

    def test_representative_is_one_real_unscaled_row(self):
        report = self.compile()
        observed = tool.validate_snapshot(self.snapshot, self.catalog)
        self.assertIn(report["representative_observed_row"], observed)
        self.assertIn(report["representative_observed_row"]["source_csv_line"], report["selection"]["source_csv_lines"])

    def test_quantiles_are_explicit_linear_interpolation(self):
        self.assertAlmostEqual(tool.quantile([10, 20], .05), 10.5)
        self.assertAlmostEqual(tool.quantile([20, 10], .95), 19.5)

    def test_import_synthetic_source_proves_units_and_source_checks(self):
        raw = b"Gender,Age,SubjectId,stature\nFemale,21,1,1600\nFemale,22,2,1700\n"
        catalog = {"ansur": {"source_id": "TEST_ONLY", "csv_sha256": sha256(raw).hexdigest(), "length_units": "mm", "expected_rows": 2},
                   "measurements": {"stature": {"female_mean_cm": 165}}}
        result = tool.import_csv(raw, catalog)
        self.assertEqual(result["rows"], [[2, 21, 1600.0], [3, 22, 1700.0]])
        self.assertNotIn("SubjectId", result["columns"])
        with self.assertRaisesRegex(ValueError, "SHA-256"):
            tool.import_csv(raw + b"\n", catalog)
        catalog["measurements"]["stature"]["female_mean_cm"] = 1650
        with self.assertRaisesRegex(ValueError, "mean disagrees"):
            tool.import_csv(raw, catalog)

    def test_import_rejects_bad_rows_even_with_updated_hash(self):
        for line in ("Male,21,1,1600", "Female,21,1,nan", "Female,21,1,-1"):
            raw = ("Gender,Age,SubjectId,stature\n" + line + "\n").encode()
            catalog = {"ansur": {"source_id": "TEST_ONLY", "csv_sha256": sha256(raw).hexdigest(), "length_units": "mm", "expected_rows": 1},
                       "measurements": {"stature": {"female_mean_cm": 160}}}
            with self.assertRaises(ValueError):
                tool.import_csv(raw, catalog)

    def test_cli_is_deterministic_and_does_not_modify_inputs(self):
        before = {path: path.read_bytes() for path in (
            tool.PROJECT / "profiles/t56_domestic_frame.json",
            tool.PROJECT / "requirements/geometry_register.json")}
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp) / "specs.json"
            command = [sys.executable, "-S", str(Path(tool.__file__).resolve()), "--output", str(output)]
            subprocess.run(command, check=True, cwd=temp, capture_output=True)
            first = output.read_bytes()
            subprocess.run(command, check=True, cwd=temp, capture_output=True)
            self.assertEqual(first, output.read_bytes())
        for path, content in before.items():
            self.assertEqual(content, path.read_bytes())


if __name__ == "__main__":
    unittest.main()
