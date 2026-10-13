"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'Talus',
    bone_type = 'short',
    location = {'region': 'tarsal', 'proximal_connection': 'tibia/fibula', 'distal_connection': 'metatarsals'},
    articulations = [],
    # PHY-DIM-072: unscaled study statistics; whole-bone unknowns remain null.
    dimensions = {
        'length_mm': 49.15,
        'thickness_mm': 24.41,
        'width_mm': 36.79,
    },
    function = ['ankle motion'],
    notable_features = [],
    developmental_notes = 'ossification varies',
    variations = '',
    unique_id = 'BONE_TALUS_L',
    visual_reference = None,
    embodiment = "virtual",
    geometry = {},
    physics = {"mass_kg": None},
    references = ['https://pmc.ncbi.nlm.nih.gov/articles/PMC6584390/', 'references/dim072/catalog.json#/candidates/24'],
    metric_sources = {
        'length_mm': 'references/dim072/catalog.json#/candidates/24/fields/length_mm',
        'thickness_mm': 'references/dim072/catalog.json#/candidates/24/fields/thickness_mm',
        'width_mm': 'references/dim072/catalog.json#/candidates/24/fields/width_mm',
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
