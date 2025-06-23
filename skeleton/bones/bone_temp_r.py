"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'Temporal',
    bone_type = 'irregular',
    location = {'region': 'skull', 'proximal_connection': 'parietal', 'distal_connection': 'mandible'},
    articulations = [{'bone': 'mandible', 'joint_type': 'hinge'}],
    dimensions = {'length_cm': 7.0, 'width_cm': 7.0, 'thickness_cm': 0.5},
    function = ['houses ear'],
    notable_features = ['mastoid'],
    developmental_notes = 'complex ossification',
    variations = '',
    unique_id = 'BONE_TEMP_R',
    visual_reference = None,
    embodiment = "virtual",
    geometry = {'shape': 'box', 'length_cm': 7.0, 'width_cm': 7.0, 'thickness_cm': 0.5},
)


def set_embodiment(state, material=None):
    """Update embodiment for this bone."""
    bone.set_embodiment(state, material)

def self_state():
    """Return the bone's current state."""
    return bone.self_state()
