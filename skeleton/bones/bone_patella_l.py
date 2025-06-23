"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'Patella',
    bone_type = 'sesamoid',
    location = {'region': 'knee', 'proximal_connection': 'femur', 'distal_connection': 'tibia'},
    articulations = [],
    dimensions = {'length_cm': None, 'width_cm': None, 'thickness_cm': None},
    function = ['protect knee joint'],
    notable_features = [],
    developmental_notes = 'ossifies within quadriceps tendon',
    variations = '',
    unique_id = 'BONE_PATELLA_L',
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
