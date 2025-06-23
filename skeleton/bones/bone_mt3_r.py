"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'Metatarsal 3',
    bone_type = 'long',
    location = {'region': 'foot', 'proximal_connection': 'tarsals', 'distal_connection': 'phalanges'},
    articulations = [],
    dimensions = {'length_cm': None, 'width_cm': None, 'thickness_cm': None},
    function = ['arch support'],
    notable_features = [],
    developmental_notes = 'growth at distal end',
    variations = '',
    unique_id = 'BONE_MT3_R',
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
