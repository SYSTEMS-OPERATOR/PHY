"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'Hyoid',
    bone_type = 'sesamoid',
    location = {'region': 'neck', 'proximal_connection': '', 'distal_connection': ''},
    articulations = [],
    dimensions = {'length_cm': 2.5, 'width_cm': 3.5, 'thickness_cm': 1.0},
    function = ['supports tongue'],
    notable_features = [],
    developmental_notes = 'ossifies from two pairs of horns',
    variations = '',
    unique_id = 'BONE_HYOID',
    visual_reference = None,
    embodiment = "virtual",
    geometry = {'shape': 'box', 'length_cm': 2.5, 'width_cm': 3.5, 'thickness_cm': 1.0},
)


def set_embodiment(state, material=None):
    """Update embodiment for this bone."""
    bone.set_embodiment(state, material)

def self_state():
    """Return the bone's current state."""
    return bone.self_state()
