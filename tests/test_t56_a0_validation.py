"""A0 calculation, evidence-boundary, and convergence regression checks."""
import importlib.util
import json
from pathlib import Path
import unittest

from PROJECTS.T56_CARBON.tools.check_convergence import audit

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "PROJECTS" / "T56_CARBON"
REQ = PROJECT / "requirements"
PACKET = PROJECT / "fabrication" / "a0_packet"
spec = importlib.util.spec_from_file_location("a0_load_cases", PROJECT / "tools" / "a0_load_cases.py")
load_cases = importlib.util.module_from_spec(spec)
spec.loader.exec_module(load_cases)


class T56A0ValidationTest(unittest.TestCase):
    def load(self, path):
        return json.loads(path.read_text(encoding="utf-8"))

    def test_all_six_calculations_match_register_criteria(self):
        result = load_cases.calculate()
        register = self.load(REQ / "load_case_register.json")
        rows = {row["id"]: row for row in register["load_cases"]}
        self.assertEqual(set(result["cases"]), {
            "LC-SHO-001", "LC-SHO-002", "LC-SHO-003",
            "LC-SHO-004", "LC-SHO-005", "LC-MNT-001",
        })
        for case_id, case in result["cases"].items():
            with self.subTest(case_id=case_id):
                self.assertEqual(case["acceptance_criteria"], rows[case_id]["acceptance_criteria"])
                self.assertFalse(case["measured_evidence"])
                self.assertNotEqual(rows[case_id]["status"], "approved")

    def test_critical_closed_form_margins_clear_design_thresholds(self):
        cases = load_cases.calculate()["cases"]
        self.assertGreater(cases["LC-SHO-003"]["results"]["tube_yield_margin_x"], 4)
        self.assertGreater(cases["LC-SHO-004"]["results"]["shaft_yield_margin_x"], 3)
        self.assertLessEqual(cases["LC-SHO-004"]["results"]["tube_stress_at_design_energy_MPa"], 120)
        self.assertLessEqual(
            cases["LC-SHO-005"]["results"]["degraded_single_fault_catch_energy_J"],
            cases["LC-SHO-005"]["acceptance_criteria"]["single_fault_maximum_catch_energy_J"],
        )

    def test_templates_cannot_be_mistaken_for_evidence(self):
        for name in ("inspection_record_template.json", "bench_test_record_template.json"):
            record = self.load(PACKET / "procedures" / name)
            self.assertEqual(record["status"], "not_run_template_only")
            self.assertFalse(record["measured_evidence"])
        closure = self.load(REQ / "single_side_article_closure.json")
        for package in closure["article_evidence_packages"]:
            self.assertNotEqual(package["status"], "approved")
            self.assertTrue(package["evidence_refs"])

    def test_gate_closes_inputs_but_not_unmeasured_qualification(self):
        report = audit("single-side")
        self.assertTrue(report["schema_valid"], report)
        self.assertFalse(report["article_ready"])
        self.assertEqual(report["counts"]["open_mission_inputs_in_scope"], 0)
        self.assertEqual(report["counts"]["open_geometry_parameters_in_scope"], 0)
        self.assertEqual(report["counts"]["approved_load_cases_in_scope"], 0)
        self.assertEqual(report["counts"]["approved_article_evidence_packages"], 0)
        self.assertEqual(len(report["blockers"]), 11)
        self.assertFalse(any("no numeric acceptance criteria" in item for item in report["blockers"]))

    def test_procedure_covers_every_article_case(self):
        procedure = (PACKET / "procedures" / "A0_BUILD_INSPECT_TEST.md").read_text(encoding="utf-8")
        closure = self.load(REQ / "single_side_article_closure.json")
        for case_id in closure["required_load_cases"]:
            self.assertIn(case_id, procedure)
        self.assertIn("NO TESTS RECORDED", procedure)


if __name__ == "__main__":
    unittest.main()
