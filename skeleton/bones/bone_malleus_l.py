"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'Malleus',
    bone_type = 'irregular',
    location = {'region': 'middle ear', 'proximal_connection': '', 'distal_connection': ''},
    articulations = [],
    dimensions = {'length_cm': 0.8, 'width_cm': 0.4, 'thickness_cm': 0.3},
    function = ['sound transmission'],
    notable_features = [],
    developmental_notes = 'ossify early',
    variations = '',
    unique_id = 'BONE_MALLEUS_L',
    visual_reference = None,
    geometry = {'shape': 'box', 'length_cm': 0.8, 'width_cm': 0.4, 'thickness_cm': 0.3},
)
