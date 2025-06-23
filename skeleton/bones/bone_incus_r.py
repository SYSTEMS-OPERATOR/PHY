"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'Incus',
    bone_type = 'irregular',
    location = {'region': 'middle ear', 'proximal_connection': '', 'distal_connection': ''},
    articulations = [],
    dimensions = {'length_cm': 0.8, 'width_cm': 0.4, 'thickness_cm': 0.3},
    function = ['sound transmission'],
    notable_features = [],
    developmental_notes = 'ossify early',
    variations = '',
    unique_id = 'BONE_INCUS_R',
    visual_reference = None,
    embodiment = "virtual",
    geometry = {'shape': 'box', 'length_cm': 0.8, 'width_cm': 0.4, 'thickness_cm': 0.3},
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
