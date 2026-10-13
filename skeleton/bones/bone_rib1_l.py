"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'Rib 1',
    bone_type = 'flat',
    location = {'region': 'thorax', 'proximal_connection': 'thoracic vertebrae', 'distal_connection': 'sternum'},
    articulations = [],
    # PHY-DIM-072: unscaled study statistics; whole-bone unknowns remain null.
    dimensions = {
        'length_mm': None,
        'midshaft_thickness_mm': 3.2,
        'midshaft_width_mm': 17.0,
        'thickness_mm': None,
        'tubercle_ventral_arc_mm': 98.6,
        'tubercle_ventral_chord_mm': 79.6,
        'width_mm': None,
    },
    function = ['protect thoracic cavity'],
    notable_features = [],
    developmental_notes = 'develop from costal cartilage',
    variations = '',
    unique_id = 'BONE_RIB1_L',
    visual_reference = None,
    embodiment = "virtual",
    geometry = {},
    physics = {"mass_kg": None},
    references = ['https://pmc.ncbi.nlm.nih.gov/articles/PMC9005670/', 'references/dim072/catalog.json#/candidates/32'],
    metric_sources = {
        'midshaft_thickness_mm': 'references/dim072/catalog.json#/candidates/32/fields/midshaft_thickness_mm',
        'midshaft_width_mm': 'references/dim072/catalog.json#/candidates/32/fields/midshaft_width_mm',
        'tubercle_ventral_arc_mm': 'references/dim072/catalog.json#/candidates/32/fields/tubercle_ventral_arc_mm',
        'tubercle_ventral_chord_mm': 'references/dim072/catalog.json#/candidates/32/fields/tubercle_ventral_chord_mm',
    },
    manufacturing_notes = ['Source-backed reference spans only; no registration to SOPHY, qualified surface, material, joint or mounting interface. See dimension evidence catalog.', 'Curved partial arc and local shaft spans are not a whole-rib bounding box or full rib length.'],
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
