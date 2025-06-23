"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'Tibia',
    bone_type = 'long',
    location = {'region': 'leg', 'proximal_connection': 'femur', 'distal_connection': 'tarsals'},
    articulations = [],
    dimensions = {'length_cm': 40.0, 'width_cm': 2.5, 'thickness_cm': 2.5},
    function = ['primary weight bearing'],
    notable_features = [],
    developmental_notes = 'tibial tuberosity prominent',
    variations = '',
    unique_id = 'BONE_TIBIA_L',
    visual_reference = None,
    embodiment = "virtual",
    geometry = {'shape': 'box', 'length_cm': 40.0, 'width_cm': 2.5, 'thickness_cm': 2.5},
)


def set_embodiment(state, material=None):
    """Update embodiment for this bone."""
    bone.set_embodiment(state, material)

def self_state():
    """Return the bone's current state."""
    return bone.self_state()
