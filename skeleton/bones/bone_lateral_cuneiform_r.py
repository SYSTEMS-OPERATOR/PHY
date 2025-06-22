"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'Lateral Cuneiform',
    bone_type = 'short',
    location = {'region': 'tarsal', 'proximal_connection': 'tibia/fibula', 'distal_connection': 'metatarsals'},
    articulations = [],
    dimensions = {'length_cm': None, 'width_cm': None, 'thickness_cm': None},
    function = ['ankle motion'],
    notable_features = [],
    developmental_notes = 'ossification varies',
    variations = '',
    unique_id = 'BONE_LATERAL_CUNEIFORM_R',
    visual_reference = None,
    geometry = {},
)
