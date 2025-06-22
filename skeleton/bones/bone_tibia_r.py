"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'Tibia',
    bone_type = 'long',
    location = {'region': 'leg', 'proximal_connection': 'femur', 'distal_connection': 'tarsals'},
    articulations = [],
    dimensions = {'length_cm': 40.0, 'width_cm': 2.5, 'thickness_cm': 2.5},
    function = ['primary weight bearing'],
    notable_features = [],
    developmental_notes = 'tibial tuberosity prominent',
    variations = '',
    unique_id = 'BONE_TIBIA_R',
    visual_reference = None,
    geometry = {'shape': 'box', 'length_cm': 40.0, 'width_cm': 2.5, 'thickness_cm': 2.5},
)
