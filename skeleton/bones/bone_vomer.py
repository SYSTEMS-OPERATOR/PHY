"""Anatomical identity only; physical measurements and interfaces unresolved."""
from ..base import BoneSpec

bone = BoneSpec(
    name='Vomer',
    latin_name="unresolved",
    bone_type="unclassified",
    location={"region": "skull", "side": 'midline'},
    articulations=[],
    dimensions={"length_mm": None, "width_mm": None, "thickness_mm": None},
    function=[],
    notable_features=[],
    developmental_notes="unresolved",
    variations="unresolved",
    unique_id='BONE_VOMER',
    geometry={},
    material={"name": "bone", "density": None},
    physics={"mass_kg": None, "inertia_kg_m2": None},
    connections={"parent": None, "children": []},
    joint_interfaces=[],
    mount_points=[],
    manufacturing_notes=[
        "Identity-only source record; no measured dimensions, surfaces, material behavior or interfaces.",
        "Reference supports anatomical identity only; see references/facial_identity_provenance.json.",
        "Default tolerance is a BODY policy placeholder, not a qualified manufacturing fit.",
    ],
    references=['https://openstax.org/books/anatomy-and-physiology-2e/pages/7-2-the-skull'],
    source_ids=['OPENSTAX_AP2E_7_2_FACIAL_IDENTITY'],
)
