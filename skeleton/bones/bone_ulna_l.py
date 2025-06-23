"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'Ulna',
    bone_type = 'long',
    location = {'region': 'forearm', 'proximal_connection': 'humerus', 'distal_connection': 'carpals'},
    articulations = [],
    dimensions = {'length_cm': 25.0, 'width_cm': 1.5, 'thickness_cm': 1.5},
    function = ['forearm hinge'],
    notable_features = [],
    developmental_notes = 'olecranon process',
    variations = '',
    unique_id = 'BONE_ULNA_L',
    visual_reference = None,
    embodiment = "virtual",
    geometry = {'shape': 'box', 'length_cm': 25.0, 'width_cm': 1.5, 'thickness_cm': 1.5},
)


def set_embodiment(state, material=None):
    """Update embodiment for this bone."""
    bone.set_embodiment(state, material)

def self_state():
    """Return the bone's current state."""
    return bone.self_state()
