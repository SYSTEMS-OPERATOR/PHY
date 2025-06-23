import os
import sys
import unittest
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from skeleton.base import BoneSpec
from skeleton.field import SkeletonField

class FaultNotifyTest(unittest.TestCase):
    def test_notify_fault_propagation(self):
        b1 = BoneSpec(name='A', bone_type='long', location={}, articulations=[],
                      dimensions={'length_cm':1,'width_cm':1,'thickness_cm':1},
                      function=[], notable_features=[], developmental_notes='',
                      variations='', unique_id='A')
        b2 = BoneSpec(name='B', bone_type='long', location={}, articulations=[],
                      dimensions={'length_cm':1,'width_cm':1,'thickness_cm':1},
                      function=[], notable_features=[], developmental_notes='',
                      variations='', unique_id='B')
        b1.entangle(b2)
        field = SkeletonField([b1, b2])
        b1.state_faults.append('test fault')
        b1.notify_fault(field)
        self.assertIn('Neighbor A fault: test fault', b2.state_faults)

if __name__ == '__main__':
    unittest.main()
