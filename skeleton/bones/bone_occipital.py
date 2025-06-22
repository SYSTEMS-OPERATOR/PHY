"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'Occipital',
    bone_type = 'flat',
    location = {'region': 'skull', 'proximal_connection': 'parietal', 'distal_connection': 'vertebral column'},
    articulations = [{'bone': 'atlas', 'joint_type': 'condyloid'}],
    dimensions = {'length_cm': 12.0, 'width_cm': 11.0, 'thickness_cm': 0.6},
    function = ['protects cerebellum'],
    notable_features = ['foramen magnum'],
    developmental_notes = 'multiple ossification centers',
    variations = '',
    unique_id = 'BONE_OCCIPITAL',
    visual_reference = None,
    geometry = {'shape': 'box', 'length_cm': 12.0, 'width_cm': 11.0, 'thickness_cm': 0.6},
)
