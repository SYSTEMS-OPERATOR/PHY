"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'Triquetrum',
    bone_type = 'short',
    location = {'region': 'carpal', 'proximal_connection': 'radius/ulna', 'distal_connection': 'metacarpals'},
    articulations = [],
    dimensions = {'length_cm': None, 'width_cm': None, 'thickness_cm': None},
    function = ['wrist flexibility'],
    notable_features = [],
    developmental_notes = 'carpal ossification varies',
    variations = '',
    unique_id = 'BONE_TRIQUETRUM_R',
    visual_reference = None,
    geometry = {},
)
