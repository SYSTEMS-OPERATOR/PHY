"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'Medial Cuneiform',
    bone_type = 'short',
    location = {'region': 'tarsal', 'proximal_connection': 'tibia/fibula', 'distal_connection': 'metatarsals'},
    articulations = [],
    dimensions = {'length_cm': None, 'width_cm': None, 'thickness_cm': None},
    function = ['ankle motion'],
    notable_features = [],
    developmental_notes = 'ossification varies',
    variations = '',
    unique_id = 'BONE_MEDIAL_CUNEIFORM_L',
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
