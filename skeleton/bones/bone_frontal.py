"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'Frontal',
    bone_type = 'flat',
    location = {'region': 'skull', 'proximal_connection': '', 'distal_connection': ''},
    articulations = [{'bone': 'parietal', 'joint_type': 'suture'}],
    dimensions = {'length_cm': 13.5, 'width_cm': 14.5, 'thickness_cm': 0.6},
    function = ['forms forehead'],
    notable_features = ['supraorbital notch'],
    developmental_notes = 'ossifies from two centers',
    variations = 'metopic suture persistence',
    unique_id = 'BONE_FRONTAL',
    visual_reference = None,
    embodiment = "virtual",
    geometry = {'shape': 'box', 'length_cm': 13.5, 'width_cm': 14.5, 'thickness_cm': 0.6},
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
