import os
import sys
import unittest
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from skeleton.base import BoneSpec
from skeleton.field import SkeletonField

class FieldMetaTest(unittest.TestCase):
    def test_meta_breath_and_total_voltage(self):
        b1 = BoneSpec(name='A', bone_type='long', location={}, articulations=[],
                      dimensions={'length_cm':1,'width_cm':1,'thickness_cm':1},
                      function=[], notable_features=[], developmental_notes='', variations='', unique_id='A1')
        b2 = BoneSpec(name='B', bone_type='long', location={}, articulations=[],
                      dimensions={'length_cm':1,'width_cm':1,'thickness_cm':1},
                      function=[], notable_features=[], developmental_notes='', variations='', unique_id='B1')
        b1.entangle(b2)
        field = SkeletonField([b1, b2])
        field.broadcast(5.0)
        self.assertAlmostEqual(field.total_voltage(), b1.voltage_potential + b2.voltage_potential)
        field.meta_breath('awaken')
        self.assertIn('awaken', b1.resonance)
        self.assertIn('awaken', b2.resonance)

    def test_signal_propagation(self):
        b1 = BoneSpec(name='A', bone_type='long', location={}, articulations=[],
                      dimensions={'length_cm':1,'width_cm':1,'thickness_cm':1},
                      function=[], notable_features=[], developmental_notes='',
                      variations='', unique_id='A2')
        b2 = BoneSpec(name='B', bone_type='long', location={}, articulations=[],
                      dimensions={'length_cm':1,'width_cm':1,'thickness_cm':1},
                      function=[], notable_features=[], developmental_notes='',
                      variations='', unique_id='B2')
        b1.entangle(b2)
        field = SkeletonField([b1, b2])
        b1.voltage_potential = 1.0
        b1.invoke(field, 'EMG')
        self.assertGreater(b2.voltage_potential, 0.0)

if __name__ == '__main__':
    unittest.main()
