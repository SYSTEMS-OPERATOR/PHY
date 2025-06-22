"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'Metatarsal 5',
    bone_type = 'long',
    location = {'region': 'foot', 'proximal_connection': 'tarsals', 'distal_connection': 'phalanges'},
    articulations = [],
    dimensions = {'length_cm': None, 'width_cm': None, 'thickness_cm': None},
    function = ['arch support'],
    notable_features = [],
    developmental_notes = 'growth at distal end',
    variations = '',
    unique_id = 'BONE_MT5_R',
    visual_reference = None,
    geometry = {},
)
