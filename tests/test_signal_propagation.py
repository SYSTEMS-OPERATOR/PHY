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
        self.assertEqual(b1.voltage_potential, 0.0)
        self.assertEqual(b2.voltage_potential, 2.0)
        self.assertAlmostEqual(field.total_voltage(), b1.voltage_potential + b2.voltage_potential)

    @staticmethod
    def linked_field(names, edges):
        bones = {
            name: BoneSpec(
                name=name, unique_id=name, bone_type='long', location={},
                articulations=[], dimensions={}, function=[], notable_features=[],
                developmental_notes='synthetic signal fixture', variations='',
            )
            for name in names
        }
        for a, b in edges:
            bones[a].entangle(bones[b])
        return SkeletonField(list(bones.values())), bones

    def test_cycle_delivers_once_per_recipient_without_echoing_to_origin(self):
        field, bones = self.linked_field('ABC', [('A', 'B'), ('B', 'C'), ('C', 'A')])
        bones['A'].voltage_potential = 1.0
        field.propagate(bones['A'].domain_id, {'voltage': 4.0, 'type': 'EMG'})
        self.assertEqual(field.summary(), {'A': 1.0, 'B': 2.0, 'C': 2.0})

    def test_diamond_does_not_redeliver_through_a_second_path(self):
        field, bones = self.linked_field(
            'ABCDE', [('A', 'B'), ('A', 'C'), ('B', 'D'), ('C', 'D')])
        field.propagate(bones['A'].domain_id, {'voltage': 4.0, 'type': 'EMG'})
        self.assertEqual(field.summary(), {'A': 0.0, 'B': 2.0, 'C': 2.0, 'D': 2.0, 'E': 0.0})

    def test_each_propagation_gets_a_fresh_visit_set_and_ignores_dangling_links(self):
        field, bones = self.linked_field('AB', [('A', 'A'), ('A', 'B')])
        bones['A'].entanglement_links.append('MISSING')
        signal = {'voltage': 4.0, 'type': 'EMG'}
        field.propagate(bones['A'].domain_id, signal)
        self.assertEqual(field.summary(), {'A': 0.0, 'B': 2.0})
        field.propagate(bones['A'].domain_id, signal)
        self.assertEqual(field.summary(), {'A': 0.0, 'B': 3.0})
        field.propagate('MISSING', signal)
        self.assertEqual(field.summary(), {'A': 0.0, 'B': 3.0})

if __name__ == '__main__':
    unittest.main()
