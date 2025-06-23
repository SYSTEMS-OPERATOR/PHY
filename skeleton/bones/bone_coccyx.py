"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'Coccyx',
    bone_type = 'irregular',
    location = {'region': 'coccyx', 'proximal_connection': 'sacrum', 'distal_connection': ''},
    articulations = [],
    dimensions = {'length_cm': None, 'width_cm': None, 'thickness_cm': None},
    function = ['ligament attachment'],
    notable_features = [],
    developmental_notes = 'fusion of rudimentary vertebrae',
    variations = '',
    unique_id = 'BONE_COCCYX',
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
