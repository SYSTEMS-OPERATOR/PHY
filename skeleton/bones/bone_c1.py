"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'C1',
    bone_type = 'irregular',
    location = {'region': 'cervical vertebrae', 'proximal_connection': 'skull', 'distal_connection': 'C2'},
    articulations = [],
    dimensions = {'length_cm': None, 'width_cm': None, 'thickness_cm': None},
    function = ['support head'],
    notable_features = [],
    developmental_notes = 'centers fuse during adolescence',
    variations = '',
    unique_id = 'BONE_C1',
    visual_reference = None,
    geometry = {},
)
