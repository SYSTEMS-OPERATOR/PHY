"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'Femur',
    bone_type = 'long',
    location = {'region': 'thigh', 'proximal_connection': 'hip', 'distal_connection': 'tibia'},
    articulations = [],
    dimensions = {'length_cm': 48.0, 'width_cm': 2.8, 'thickness_cm': 2.8},
    function = ['weight bearing'],
    notable_features = ['greater trochanter'],
    developmental_notes = 'epiphyseal plates fuse in adulthood',
    variations = '',
    unique_id = 'BONE_FEMUR_R',
    visual_reference = None,
    embodiment = "virtual",
    geometry = {'shape': 'box', 'length_cm': 48.0, 'width_cm': 2.8, 'thickness_cm': 2.8},
)


def set_embodiment(state, material=None):
    """Update embodiment for this bone."""
    bone.set_embodiment(state, material)

def self_state():
    """Return the bone's current state."""
    return bone.self_state()
