"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'Patella',
    bone_type = 'sesamoid',
    location = {'region': 'knee', 'proximal_connection': 'femur', 'distal_connection': 'tibia'},
    articulations = [],
    # PHY-DIM-072: unscaled study statistics; whole-bone unknowns remain null.
    dimensions = {
        'length_mm': 41.05,
        'thickness_mm': 22.56,
        'width_mm': 42.71,
    },
    function = ['protect knee joint'],
    notable_features = [],
    developmental_notes = 'ossifies within quadriceps tendon',
    variations = '',
    unique_id = 'BONE_PATELLA_R',
    visual_reference = None,
    embodiment = "virtual",
    geometry = {},
    physics = {"mass_kg": None},
    references = ['https://sajs.co.za/article/download/4002/5775', 'references/dim072/catalog.json#/candidates/31'],
    metric_sources = {
        'length_mm': 'references/dim072/catalog.json#/candidates/31/fields/length_mm',
        'thickness_mm': 'references/dim072/catalog.json#/candidates/31/fields/thickness_mm',
        'width_mm': 'references/dim072/catalog.json#/candidates/31/fields/width_mm',
    },
    manufacturing_notes = ['Source-backed reference spans only; no registration to SOPHY, qualified surface, material, joint or mounting interface. See dimension evidence catalog.'],
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
