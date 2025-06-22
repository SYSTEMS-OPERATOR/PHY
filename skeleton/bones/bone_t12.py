"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'T12',
    bone_type = 'irregular',
    location = {'region': 'thoracic vertebrae', 'proximal_connection': 'T11', 'distal_connection': 'L1'},
    articulations = [],
    dimensions = {'length_cm': None, 'width_cm': None, 'thickness_cm': None},
    function = ['rib articulation'],
    notable_features = [],
    developmental_notes = 'vertebral arch fusion in adolescence',
    variations = '',
    unique_id = 'BONE_T12',
    visual_reference = None,
    geometry = {},
)
