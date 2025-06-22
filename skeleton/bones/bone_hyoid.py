"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'Hyoid',
    bone_type = 'sesamoid',
    location = {'region': 'neck', 'proximal_connection': '', 'distal_connection': ''},
    articulations = [],
    dimensions = {'length_cm': 2.5, 'width_cm': 3.5, 'thickness_cm': 1.0},
    function = ['supports tongue'],
    notable_features = [],
    developmental_notes = 'ossifies from two pairs of horns',
    variations = '',
    unique_id = 'BONE_HYOID',
    visual_reference = None,
    geometry = {'shape': 'box', 'length_cm': 2.5, 'width_cm': 3.5, 'thickness_cm': 1.0},
)
