"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'Rib 12',
    bone_type = 'flat',
    location = {'region': 'thorax', 'proximal_connection': 'thoracic vertebrae', 'distal_connection': 'costal cartilage'},
    articulations = [],
    dimensions = {'length_cm': None, 'width_cm': None, 'thickness_cm': None},
    function = ['protect thoracic cavity'],
    notable_features = [],
    developmental_notes = 'develop from costal cartilage',
    variations = '',
    unique_id = 'BONE_RIB12_L',
    visual_reference = None,
    geometry = {},
)
