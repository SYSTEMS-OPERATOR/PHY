import os
import sys
import unittest
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from skeleton.base import BoneSpec
from skeleton.field import SkeletonField

class SignalPropagationTest(unittest.TestCase):
    def test_propagate_signal(self):
        b1 = BoneSpec(name='A', bone_type='long', location={}, articulations=[],
                      dimensions={'length_cm':1,'width_cm':1,'thickness_cm':1},
                      function=[], notable_features=[], developmental_notes='',
                      variations='', unique_id='A1')
        b2 = BoneSpec(name='B', bone_type='long', location={}, articulations=[],
                      dimensions={'length_cm':1,'width_cm':1,'thickness_cm':1},
                      function=[], notable_features=[], developmental_notes='',
                      variations='', unique_id='B1')
        b1.entangle(b2)
        field = SkeletonField([b1, b2])
        signal = {'voltage': 4.0, 'type': 'EMG'}
        field.propagate(b1.domain_id, signal)
        self.assertGreater(b2.voltage_potential, 0)
        self.assertAlmostEqual(field.total_voltage(), b1.voltage_potential + b2.voltage_potential)

if __name__ == '__main__':
    unittest.main()
