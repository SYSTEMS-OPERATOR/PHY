"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'Radius',
    bone_type = 'long',
    location = {'region': 'forearm', 'proximal_connection': 'humerus', 'distal_connection': 'carpals'},
    articulations = [],
    dimensions = {'length_cm': 24.0, 'width_cm': 1.5, 'thickness_cm': 1.5},
    function = ['forearm rotation'],
    notable_features = [],
    developmental_notes = 'radius head ossifies separately',
    variations = '',
    unique_id = 'BONE_RADIUS_L',
    visual_reference = None,
    geometry = {'shape': 'box', 'length_cm': 24.0, 'width_cm': 1.5, 'thickness_cm': 1.5},
)
