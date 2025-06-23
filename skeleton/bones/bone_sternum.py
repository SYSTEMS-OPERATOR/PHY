"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'Sternum',
    bone_type = 'flat',
    location = {'region': 'thorax', 'proximal_connection': 'clavicle', 'distal_connection': 'ribs'},
    articulations = [],
    dimensions = {'length_cm': None, 'width_cm': None, 'thickness_cm': None},
    function = ['protects thoracic organs'],
    notable_features = ['manubrium', 'xiphoid'],
    developmental_notes = 'fuses from sternebrae',
    variations = '',
    unique_id = 'BONE_STERNUM',
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
