"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'T5',
    bone_type = 'irregular',
    location = {'region': 'thoracic vertebrae', 'proximal_connection': 'T4', 'distal_connection': 'T6'},
    articulations = [],
    dimensions = {'length_cm': None, 'width_cm': None, 'thickness_cm': None},
    function = ['rib articulation'],
    notable_features = [],
    developmental_notes = 'vertebral arch fusion in adolescence',
    variations = '',
    unique_id = 'BONE_T5',
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
