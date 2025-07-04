import unittest
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from skeleton.base import BoneSpec
from skeleton.bones import bone_c1

class BoneStateTest(unittest.TestCase):
    def setUp(self):
        self.bone = BoneSpec(
            name='Test',
            bone_type='long',
            location={},
            articulations=[],
            dimensions={'length_cm': 10.0, 'width_cm': 2.0, 'thickness_cm': 1.0},
            function=[],
            notable_features=[],
            developmental_notes='',
            variations='',
            unique_id='TEST1'
        )

    def test_fault_on_mass_virtual(self):
        self.assertIsNone(self.bone.mass_kg())
        self.assertFalse(self.bone.is_healthy())
        self.assertTrue(any('Mass error' in f for f in self.bone.state_faults))

    def test_update_and_clear_faults(self):
        self.bone.clear_faults()
        self.bone.set_embodiment('physical', {'density': 1000})
        mass = self.bone.mass_kg()
        self.assertIsNotNone(mass)
        self.bone.update_state(position=(1,2,3), load=(10,0,0))
        state = self.bone.current_state()
        self.assertEqual(state['position'], (1,2,3))
        self.assertTrue(self.bone.is_healthy())

    def test_module_wrappers(self):
        bone_c1.clear_faults()
        bone_c1.set_embodiment('virtual')
        self.assertIsNone(bone_c1.bone.mass_kg())
        self.assertFalse(bone_c1.is_healthy())
        bone_c1.clear_faults()
        bone_c1.update_state(position=(5,5,5), torsion=(1,0,0))
        state = bone_c1.current_state()
        self.assertEqual(state['position'], (5,5,5))
        self.assertTrue(bone_c1.is_healthy())

if __name__ == '__main__':
    unittest.main()
