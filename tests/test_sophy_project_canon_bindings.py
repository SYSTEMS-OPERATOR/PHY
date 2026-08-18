from __future__ import annotations

import json
from pathlib import Path
import unittest

from skeleton.canon import instantiate_sophy_canon


REPO_ROOT = Path(__file__).resolve().parents[1]


class SophyProjectCanonBindingTest(unittest.TestCase):
    def _load(self, relative_path: str):
        return json.loads((REPO_ROOT / relative_path).read_text(encoding="utf-8"))

    def test_redwood_binds_scale_but_not_identity_law(self):
        profile = self._load("PROJECTS/REDWOOD/profiles/adult_female_21_28.json")
        reference = profile["canon_reference"]
        canon = instantiate_sophy_canon(
            reference["height_mm"],
            canon_version=reference["canon_version"],
        )
        self.assertEqual(profile["arm_span_mm"], canon.arm_span_mm)
        self.assertEqual(profile["authority"], "project_local_non_canonical")
        self.assertFalse(profile["bilateral_symmetry_policy"]["canonical_asymmetry_allowed"])
        self.assertEqual(
            profile["geometry_classification"]["primary_lengths_mm"],
            "legacy_provisional_anthropometric_engineering_reference",
        )

    def test_t56_binds_1676_4_mm_canon(self):
        profile = self._load("PROJECTS/T56_CARBON/profiles/t56_domestic_frame.json")
        reference = profile["canon_reference"]
        canon = instantiate_sophy_canon(
            reference["height_mm"],
            canon_version=reference["canon_version"],
        )
        self.assertEqual(canon.height_mm, 1676.4)
        self.assertEqual(profile["arm_span_mm"], canon.arm_span_mm)
        self.assertFalse(profile["proportion_policy"]["canonical_asymmetry_allowed"])
        self.assertTrue(profile["proportion_policy"]["mechanical_calibration_may_be_independent"])


if __name__ == "__main__":
    unittest.main()
