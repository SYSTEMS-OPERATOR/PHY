import os
import sys
import unittest
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from skeleton.bones import load_bones

class DatasetLinkTest(unittest.TestCase):
    def test_dataset_available_even_if_metrics_missing(self):
        bones = load_bones('female_21_baseline')
        # pick a bone not present in dataset
        bone = next(b for b in bones if b.name == 'C1')
        self.assertIsNotNone(bone.dataset)
        # dataset_key should be None but we should still be able to set material
        bone.set_material('Ti6Al4V')
        self.assertIn('density', bone.material)
        self.assertNotIn('No dataset available for material lookup', bone.state_faults)

if __name__ == '__main__':
    unittest.main()
