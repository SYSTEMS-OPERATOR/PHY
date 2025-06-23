"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'Sacrum',
    bone_type = 'irregular',
    location = {'region': 'sacrum', 'proximal_connection': 'L5', 'distal_connection': 'coccyx'},
    articulations = [],
    dimensions = {'length_cm': None, 'width_cm': None, 'thickness_cm': None},
    function = ['supports pelvis'],
    notable_features = [],
    developmental_notes = 'fusion of 5 sacral vertebrae',
    variations = '',
    unique_id = 'BONE_SACRUM',
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
