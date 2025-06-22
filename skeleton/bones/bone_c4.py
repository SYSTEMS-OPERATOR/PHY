"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'C4',
    bone_type = 'irregular',
    location = {'region': 'cervical vertebrae', 'proximal_connection': 'C3', 'distal_connection': 'C5'},
    articulations = [],
    dimensions = {'length_cm': None, 'width_cm': None, 'thickness_cm': None},
    function = ['support neck'],
    notable_features = [],
    developmental_notes = 'centers fuse during adolescence',
    variations = '',
    unique_id = 'BONE_C4',
    visual_reference = None,
    geometry = {},
)
