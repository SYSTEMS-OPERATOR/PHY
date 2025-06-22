from ..base import BoneSpec

bone = BoneSpec(
    name = 'Frontal',
    bone_type = 'flat',
    location = {'region': 'skull', 'proximal_connection': '', 'distal_connection': ''},
    articulations = [{'bone': 'parietal', 'joint_type': 'suture'}],
    dimensions = {'length_cm': 13.5, 'width_cm': 14.5, 'thickness_cm': 0.6},
    function = ['forms forehead'],
    notable_features = ['supraorbital notch'],
    developmental_notes = 'ossifies from two centers',
    variations = 'metopic suture persistence',
    unique_id = 'BONE_FRONTAL',
    visual_reference = None,
    geometry = {'shape': 'box', 'length_cm': 13.5, 'width_cm': 14.5, 'thickness_cm': 0.6},
)
