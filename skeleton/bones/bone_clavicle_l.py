"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'Clavicle',
    bone_type = 'long',
    location = {'region': 'shoulder', 'proximal_connection': 'sternum', 'distal_connection': 'scapula'},
    articulations = [],
    dimensions = {'length_cm': 14.0, 'width_cm': 1.0, 'thickness_cm': 1.0},
    function = ['strut for shoulder'],
    notable_features = [],
    developmental_notes = 'ossifies first among long bones',
    variations = '',
    unique_id = 'BONE_CLAVICLE_L',
    visual_reference = None,
    embodiment = "virtual",
    geometry = {'shape': 'box', 'length_cm': 14.0, 'width_cm': 1.0, 'thickness_cm': 1.0},
)


def set_embodiment(state, material=None):
    """Update embodiment for this bone."""
    bone.set_embodiment(state, material)

def self_state():
    """Return the bone's current state."""
    return bone.self_state()
