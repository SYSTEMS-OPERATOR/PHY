"""Bone module with embodiment-aware BoneSpec. Default embodiment "virtual"."""
from ..base import BoneSpec

bone = BoneSpec(
    name = 'Parietal',
    bone_type = 'flat',
    location = {'region': 'skull', 'proximal_connection': 'frontal', 'distal_connection': 'occipital'},
    articulations = [{'bone': 'frontal', 'joint_type': 'suture'}, {'bone': 'occipital', 'joint_type': 'suture'}],
    dimensions = {'length_cm': 14.0, 'width_cm': 10.0, 'thickness_cm': 0.5},
    function = ['protects brain'],
    notable_features = ['parietal eminence'],
    developmental_notes = 'intramembranous ossification',
    variations = '',
    unique_id = 'BONE_PAR_R',
    visual_reference = None,
    geometry = {'shape': 'box', 'length_cm': 14.0, 'width_cm': 10.0, 'thickness_cm': 0.5},
)
