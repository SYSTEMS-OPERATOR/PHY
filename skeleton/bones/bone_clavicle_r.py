"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'Clavicle',
    bone_type = 'long',
    location = {'region': 'shoulder', 'proximal_connection': 'sternum', 'distal_connection': 'scapula'},
    articulations = [],
    dimensions = {'length_cm': 14.0, 'width_cm': 1.0, 'thickness_cm': 1.0},
    function = ['strut for shoulder'],
    notable_features = [],
    developmental_notes = 'ossifies first among long bones',
    variations = '',
    unique_id = 'BONE_CLAVICLE_R',
    visual_reference = None,
    embodiment = "virtual",
    geometry = {'shape': 'box', 'length_cm': 14.0, 'width_cm': 1.0, 'thickness_cm': 1.0},
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


def notify_fault(field=None):
    """Notify linked bones of a fault."""
    bone.notify_fault(field)
