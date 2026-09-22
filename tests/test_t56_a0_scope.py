import json
from pathlib import Path
import unittest

from skeleton.canon import instantiate_sophy_canon


ROOT = Path(__file__).resolve().parents[1]
REQ = ROOT / "PROJECTS" / "T56_CARBON" / "requirements"


class T56A0ScopeTest(unittest.TestCase):
    def load(self, name):
        return json.loads((REQ / name).read_text(encoding="utf-8"))

    def test_scope_preserves_canon(self):
        scope = self.load("a0_article_scope.json")
        binding = scope["canon_binding"]
        canon = instantiate_sophy_canon(
            binding["height_mm"], canon_version=binding["canon_version"]
        )
        self.assertEqual(binding["height_mm"], canon.height_mm)
        self.assertEqual(binding["arm_span_mm"], canon.arm_span_mm)
        self.assertFalse(binding["mutation_permitted"])

    def test_article_mission_values_match_scope(self):
        scope = self.load("a0_article_scope.json")
        mission = self.load("mission_envelope.json")
        rows = {row["id"]: row for row in mission["required_inputs"]}
        self.assertEqual(set(scope["mission_inputs"]), {
            "ME-MASS-002", "ME-PAYLOAD-001", "ME-MOTION-001", "ME-MOTION-002",
            "ME-EXT-001", "ME-IMPACT-001", "ME-LIFE-001", "ME-SF-001",
            "ME-ACOUSTIC-001", "ME-SERVICE-001",
        })
        for record_id, expected in scope["mission_inputs"].items():
            with self.subTest(record_id=record_id):
                actual = rows[record_id]
                self.assertEqual(actual["status"], "locked")
                self.assertEqual(actual["value"], expected["value"])
                self.assertEqual(actual["units"], expected["units"])
                self.assertTrue(actual["authority"])
                evidence = actual["evidence"].lower()
                self.assertTrue(
                    any(marker in evidence for marker in ("open", "unverified", "not a measured"))
                )

    def test_scope_is_explicitly_non_load_bearing_and_unqualified(self):
        scope = self.load("a0_article_scope.json")
        self.assertFalse(scope["prototype_class"]["load_bearing"])
        self.assertIn("body_weight_support", scope["required_exclusions"])
        self.assertFalse(scope["qualification_boundary"]["design_values_are_measured_capabilities"])
        self.assertFalse(scope["qualification_boundary"]["simulation_closes_physical_evidence"])


if __name__ == "__main__":
    unittest.main()
