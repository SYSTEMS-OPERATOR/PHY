"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'Sphenoid',
    bone_type = 'irregular',
    location = {'region': 'skull', 'proximal_connection': 'frontal', 'distal_connection': 'occipital'},
    articulations = [{'bone': 'frontal', 'joint_type': 'suture'}],
    dimensions = {'length_cm': 6.0, 'width_cm': 8.0, 'thickness_cm': 0.5},
    function = ['forms skull base'],
    notable_features = ['sella turcica'],
    developmental_notes = 'cartilaginous ossification',
    variations = '',
    unique_id = 'BONE_SPHENOID',
    visual_reference = None,
    embodiment = "virtual",
    geometry = {'shape': 'box', 'length_cm': 6.0, 'width_cm': 8.0, 'thickness_cm': 0.5},
)


def set_embodiment(state, material=None):
    """Update embodiment for this bone."""
    bone.set_embodiment(state, material)

def self_state():
    """Return the bone's current state."""
    return bone.self_state()
