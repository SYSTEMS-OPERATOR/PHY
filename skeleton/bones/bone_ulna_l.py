"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'Ulna',
    bone_type = 'long',
    location = {'region': 'forearm', 'proximal_connection': 'humerus', 'distal_connection': 'carpals'},
    articulations = [],
    dimensions = {'length_cm': 25.0, 'width_cm': 1.5, 'thickness_cm': 1.5},
    function = ['forearm hinge'],
    notable_features = [],
    developmental_notes = 'olecranon process',
    variations = '',
    unique_id = 'BONE_ULNA_L',
    visual_reference = None,
    geometry = {'shape': 'box', 'length_cm': 25.0, 'width_cm': 1.5, 'thickness_cm': 1.5},
)
