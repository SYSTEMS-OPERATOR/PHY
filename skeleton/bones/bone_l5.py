"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'L5',
    bone_type = 'irregular',
    location = {'region': 'lumbar vertebrae', 'proximal_connection': 'L4', 'distal_connection': 'sacrum'},
    articulations = [],
    dimensions = {'length_cm': None, 'width_cm': None, 'thickness_cm': None},
    function = ['weight bearing'],
    notable_features = [],
    developmental_notes = 'large body for support',
    variations = '',
    unique_id = 'BONE_L5',
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
