"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'Coccyx',
    bone_type = 'irregular',
    location = {'region': 'coccyx', 'proximal_connection': 'BONE_SACRUM', 'distal_connection': ''},
    articulations = [],
    dimensions = {'length_cm': None, 'width_cm': None, 'thickness_cm': None},
    function = ['ligament attachment'],
    notable_features = ['Usually four coccygeal segments, with three to five observed; fusion is variable.', 'Sacrococcygeal and intercoccygeal interfaces vary in fusion and alignment.', 'Straight and curved lengths are different anatomical measurements.'],
    developmental_notes = 'Usually four coccygeal segments; number and extent of fusion vary.',
    variations = '',
    unique_id = 'BONE_COCCYX',
    visual_reference = None,
    embodiment = "virtual",
    geometry = {},
    references = ['https://pmc.ncbi.nlm.nih.gov/articles/PMC3631051/'],
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
