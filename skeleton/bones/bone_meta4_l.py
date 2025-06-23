"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'Metacarpal 4',
    bone_type = 'long',
    location = {'region': 'hand', 'proximal_connection': 'carpals', 'distal_connection': 'phalanges'},
    articulations = [],
    dimensions = {'length_cm': None, 'width_cm': None, 'thickness_cm': None},
    function = ['palm support'],
    notable_features = [],
    developmental_notes = 'growth plate at distal end',
    variations = '',
    unique_id = 'BONE_META4_L',
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
