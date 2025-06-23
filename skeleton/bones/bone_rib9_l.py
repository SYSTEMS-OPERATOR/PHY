"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'Rib 9',
    bone_type = 'flat',
    location = {'region': 'thorax', 'proximal_connection': 'thoracic vertebrae', 'distal_connection': 'costal cartilage'},
    articulations = [],
    dimensions = {'length_cm': None, 'width_cm': None, 'thickness_cm': None},
    function = ['protect thoracic cavity'],
    notable_features = [],
    developmental_notes = 'develop from costal cartilage',
    variations = '',
    unique_id = 'BONE_RIB9_L',
    visual_reference = None,
    embodiment = "virtual",
    geometry = {},
)


def set_embodiment(state, material=None):
    """Update embodiment for this bone."""
    bone.set_embodiment(state, material)

def self_state():
    """Return the bone's current state."""
    return bone.self_state()
