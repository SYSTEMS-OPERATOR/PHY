"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'Middle Toe Phalanx 3',
    bone_type = 'long',
    location = {'region': 'toe', 'proximal_connection': 'Proximal phalanx', 'distal_connection': 'Distal phalanx'},
    articulations = [],
    dimensions = {'length_cm': None, 'width_cm': None, 'thickness_cm': None},
    function = ['toe movement'],
    notable_features = [],
    developmental_notes = 'ossification begins distal',
    variations = '',
    unique_id = 'BONE_T_PHAL_3_2_L',
    visual_reference = None,
    geometry = {},
)
