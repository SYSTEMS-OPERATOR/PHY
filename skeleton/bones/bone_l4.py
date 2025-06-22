"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'L4',
    bone_type = 'irregular',
    location = {'region': 'lumbar vertebrae', 'proximal_connection': 'L3', 'distal_connection': 'L5'},
    articulations = [],
    dimensions = {'length_cm': None, 'width_cm': None, 'thickness_cm': None},
    function = ['weight bearing'],
    notable_features = [],
    developmental_notes = 'large body for support',
    variations = '',
    unique_id = 'BONE_L4',
    visual_reference = None,
    geometry = {},
)
