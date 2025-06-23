"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'Humerus',
    bone_type = 'long',
    location = {'region': 'upper arm', 'proximal_connection': 'scapula', 'distal_connection': 'radius/ulna'},
    articulations = [],
    dimensions = {'length_cm': 35.0, 'width_cm': 2.5, 'thickness_cm': 2.5},
    function = ['lever for arm'],
    notable_features = ['deltoid tuberosity'],
    developmental_notes = 'growth plates at both ends',
    variations = '',
    unique_id = 'BONE_HUMERUS_R',
    visual_reference = None,
    embodiment = "virtual",
    geometry = {'shape': 'box', 'length_cm': 35.0, 'width_cm': 2.5, 'thickness_cm': 2.5},
)


def set_embodiment(state, material=None):
    """Update embodiment for this bone."""
    bone.set_embodiment(state, material)

def self_state():
    """Return the bone's current state."""
    return bone.self_state()

def update_state(position=None, orientation=None, load=None, torsion=None):
    """Atomically update bone state."""
    bone.update_state(position=position, orientation=orientation,
                      load=load, torsion=torsion)

def current_state():
    """Return the bone's current state."""
    return bone.current_state()

def is_healthy():
    """Return True if the bone has no recorded faults."""
    return bone.is_healthy()

def clear_faults():
    """Clear the bone's fault log."""
    bone.clear_faults()

def report_faults():
    """Return list of recorded faults."""
    return bone.report_faults()
