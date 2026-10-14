"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'L3',
    bone_type = 'irregular',
    location = {'region': 'lumbar vertebrae', 'proximal_connection': 'L2', 'distal_connection': 'L4'},
    articulations = [],
    # PHY-DIM-072: unscaled study statistics; whole-bone unknowns remain null.
    dimensions = {
        'body_height_mm': 25.32,
        'body_lower_ap_mm': 34.25,
        'body_lower_width_mm': 50.97,
        'body_upper_ap_mm': 34.48,
        'body_upper_width_mm': 48.22,
        'length_mm': None,
        'thickness_mm': None,
        'width_mm': None,
    },
    function = ['weight bearing'],
    notable_features = [],
    developmental_notes = 'large body for support',
    variations = '',
    unique_id = 'BONE_L3',
    visual_reference = None,
    embodiment = "virtual",
    geometry = {},
    physics = {"mass_kg": None},
    references = ['https://pmc.ncbi.nlm.nih.gov/articles/PMC8917002/', 'references/dim072/catalog.json#/candidates/2'],
    metric_sources = {
        'body_height_mm': 'references/dim072/catalog.json#/candidates/2/fields/body_height_mm',
        'body_lower_ap_mm': 'references/dim072/catalog.json#/candidates/2/fields/body_lower_ap_mm',
        'body_lower_width_mm': 'references/dim072/catalog.json#/candidates/2/fields/body_lower_width_mm',
        'body_upper_ap_mm': 'references/dim072/catalog.json#/candidates/2/fields/body_upper_ap_mm',
        'body_upper_width_mm': 'references/dim072/catalog.json#/candidates/2/fields/body_upper_width_mm',
    },
    manufacturing_notes = ['Source-backed reference spans only; no registration to SOPHY, qualified surface, material, joint or mounting interface. See dimension evidence catalog.', 'Body spans do not resolve the whole vertebra length/width/thickness; posterior elements remain unmeasured.'],
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
