"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'C1',
    bone_type = 'irregular',
    location = {'region': 'cervical vertebrae', 'proximal_connection': 'skull', 'distal_connection': 'C2'},
    articulations = [],
    # PHY-DIM-072: unscaled study statistics; whole-bone unknowns remain null.
    dimensions = {
        'length_mm': None,
        'spinal_canal_ap_mm': 29.79,
        'spinal_canal_width_mm': 27.38,
        'thickness_mm': None,
        'width_mm': None,
    },
    function = ['support head'],
    notable_features = [],
    developmental_notes = 'centers fuse during adolescence',
    variations = '',
    unique_id = 'BONE_C1',
    visual_reference = None,
    embodiment = "virtual",
    geometry = {},
    physics = {"mass_kg": None},
    references = ['https://pmc.ncbi.nlm.nih.gov/articles/PMC8354927/', 'references/dim072/catalog.json#/candidates/23'],
    metric_sources = {
        'spinal_canal_ap_mm': 'references/dim072/catalog.json#/candidates/23/fields/spinal_canal_ap_mm',
        'spinal_canal_width_mm': 'references/dim072/catalog.json#/candidates/23/fields/spinal_canal_width_mm',
    },
    manufacturing_notes = ['Source-backed reference spans only; no registration to SOPHY, qualified surface, material, joint or mounting interface. See dimension evidence catalog.', 'Canal dimensions are clearances, not dimensions of solid bone or a defined mechanical cartridge.', 'Atlas has no centrum; no C1 body height invented.'],
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
