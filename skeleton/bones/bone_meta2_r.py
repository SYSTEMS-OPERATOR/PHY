"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'Metacarpal 2',
    bone_type = 'long',
    location = {'region': 'hand', 'proximal_connection': 'carpals', 'distal_connection': 'phalanges'},
    articulations = [],
    dimensions = {'length_cm': None, 'width_cm': None, 'thickness_cm': None},
    function = ['palm support'],
    notable_features = [],
    developmental_notes = 'growth plate at distal end',
    variations = '',
    unique_id = 'BONE_META2_R',
    visual_reference = None,
    geometry = {},
)
