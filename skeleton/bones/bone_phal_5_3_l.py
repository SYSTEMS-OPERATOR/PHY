"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'Distal Phalanx 5',
    bone_type = 'long',
    location = {'region': 'finger', 'proximal_connection': 'Middle phalanx', 'distal_connection': ''},
    articulations = [],
    dimensions = {'length_cm': None, 'width_cm': None, 'thickness_cm': None},
    function = ['digit movement'],
    notable_features = [],
    developmental_notes = 'ossification begins distal',
    variations = '',
    unique_id = 'BONE_PHAL_5_3_L',
    visual_reference = None,
    geometry = {},
)
