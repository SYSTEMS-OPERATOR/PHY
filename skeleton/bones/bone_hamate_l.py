"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'Hamate',
    bone_type = 'short',
    location = {'region': 'carpal', 'proximal_connection': 'radius/ulna', 'distal_connection': 'metacarpals'},
    articulations = [],
    dimensions = {'length_cm': None, 'width_cm': None, 'thickness_cm': None},
    function = ['wrist flexibility'],
    notable_features = [],
    developmental_notes = 'carpal ossification varies',
    variations = '',
    unique_id = 'BONE_HAMATE_L',
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
