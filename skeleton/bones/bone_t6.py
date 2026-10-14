"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'T6',
    bone_type = 'irregular',
    location = {'region': 'thoracic vertebrae', 'proximal_connection': 'T5', 'distal_connection': 'T7'},
    articulations = [],
    # PHY-DIM-072: unscaled study statistics; whole-bone unknowns remain null.
    dimensions = {
        'body_mid_width_mm': 22.58,
        'body_lower_width_mm': 25.33,
        'body_anterior_height_mm': 16.95,
        'body_posterior_height_mm': 18.16,
        'length_mm': None,
        'thickness_mm': None,
        'width_mm': None,
    },
    function = ['rib articulation'],
    notable_features = [],
    developmental_notes = 'vertebral arch fusion in adolescence',
    variations = '',
    unique_id = 'BONE_T6',
    visual_reference = None,
    embodiment = "virtual",
    geometry = {},
    physics = {"mass_kg": None},
    references = ['https://www.asianspinejournal.org/upload/pdf/asj-5-20.pdf', 'references/dim072/catalog.json#/candidates/11'],
    metric_sources = {
        'body_mid_width_mm': 'references/dim072/catalog.json#/candidates/11/fields/body_mid_width_mm',
        'body_lower_width_mm': 'references/dim072/catalog.json#/candidates/11/fields/body_lower_width_mm',
        'body_anterior_height_mm': 'references/dim072/catalog.json#/candidates/11/fields/body_anterior_height_mm',
        'body_posterior_height_mm': 'references/dim072/catalog.json#/candidates/11/fields/body_posterior_height_mm',
    },
    manufacturing_notes = ['Source-backed reference spans only; no registration to SOPHY, qualified surface, material, joint or mounting interface. See dimension evidence catalog.', 'Endplate heights do not qualify whole vertebral dimensions, surface shape or articulation.'],
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
