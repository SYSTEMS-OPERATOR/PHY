"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'Calcaneus',
    bone_type = 'short',
    location = {'region': 'tarsal', 'proximal_connection': 'tibia/fibula', 'distal_connection': 'metatarsals'},
    articulations = [],
    # PHY-DIM-072: unscaled study statistics; whole-bone unknowns remain null.
    dimensions = {
        'calcaneal_body_width_mm': 33.0,
        'calcaneal_lateral_wall_height_mm': 33.3,
        'length_mm': 66.2,
        'thickness_mm': None,
        'width_mm': None,
    },
    function = ['ankle motion'],
    notable_features = [],
    developmental_notes = 'ossification varies',
    variations = '',
    unique_id = 'BONE_CALCANEUS_R',
    visual_reference = None,
    embodiment = "virtual",
    geometry = {},
    physics = {"mass_kg": None},
    references = ['https://pmc.ncbi.nlm.nih.gov/articles/PMC11116927/', 'references/dim072/catalog.json#/candidates/27'],
    metric_sources = {
        'calcaneal_body_width_mm': 'references/dim072/catalog.json#/candidates/27/fields/calcaneal_body_width_mm',
        'calcaneal_lateral_wall_height_mm': 'references/dim072/catalog.json#/candidates/27/fields/calcaneal_lateral_wall_height_mm',
        'length_mm': 'references/dim072/catalog.json#/candidates/27/fields/length_mm',
    },
    manufacturing_notes = ['Source-backed reference spans only; no registration to SOPHY, qualified surface, material, joint or mounting interface. See dimension evidence catalog.', 'Whole-bone width and thickness remain unknown; localized body/wall measurements do not replace them.'],
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
