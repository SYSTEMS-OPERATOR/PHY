"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'C3',
    bone_type = 'irregular',
    location = {'region': 'cervical vertebrae', 'proximal_connection': 'C2', 'distal_connection': 'C4'},
    articulations = [],
    # PHY-DIM-072: unscaled study statistics; whole-bone unknowns remain null.
    dimensions = {
        'body_height_mm': 11.96,
        'length_mm': None,
        'spinal_canal_ap_mm': 13.85,
        'spinal_canal_width_mm': 22.26,
        'thickness_mm': None,
        'width_mm': None,
    },
    function = ['support neck'],
    notable_features = [],
    developmental_notes = 'centers fuse during adolescence',
    variations = '',
    unique_id = 'BONE_C3',
    visual_reference = None,
    embodiment = "virtual",
    geometry = {},
    physics = {"mass_kg": None},
    references = ['https://pmc.ncbi.nlm.nih.gov/articles/PMC8354927/', 'references/dim072/catalog.json#/candidates/21'],
    metric_sources = {
        'body_height_mm': 'references/dim072/catalog.json#/candidates/21/fields/body_height_mm',
        'spinal_canal_ap_mm': 'references/dim072/catalog.json#/candidates/21/fields/spinal_canal_ap_mm',
        'spinal_canal_width_mm': 'references/dim072/catalog.json#/candidates/21/fields/spinal_canal_width_mm',
    },
    manufacturing_notes = ['Source-backed reference spans only; no registration to SOPHY, qualified surface, material, joint or mounting interface. See dimension evidence catalog.', 'Canal dimensions are clearances, not dimensions of solid bone or a defined mechanical cartridge.'],
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
