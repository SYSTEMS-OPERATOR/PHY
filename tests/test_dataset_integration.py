import os
import sys
import unittest
import warnings

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from skeleton.bones import load_bones

class DatasetIntegrationTest(unittest.TestCase):
    def test_hipbone_dataset_applied(self):
        warnings.filterwarnings("ignore")
        bones = load_bones("female_21_baseline")
        hip = next(b for b in bones if b.unique_id == "BONE_HIP_L")
        self.assertEqual(hip.dataset_key, "HipBone")
        self.assertAlmostEqual(hip.dimensions.get("length_cm"), 23.0)
        self.assertEqual(hip.material.get("mass_g"), 120)

if __name__ == "__main__":
    unittest.main()
