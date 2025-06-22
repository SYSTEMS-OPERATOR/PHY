from ..base import BoneSpec

bone = BoneSpec(
    name = 'Fibula',
    bone_type = 'long',
    location = {'region': 'leg', 'proximal_connection': 'tibia', 'distal_connection': 'tarsals'},
    articulations = [],
    dimensions = {'length_cm': 40.0, 'width_cm': 1.5, 'thickness_cm': 1.5},
    function = ['ankle stability'],
    notable_features = [],
    developmental_notes = 'slender bone',
    variations = '',
    unique_id = 'BONE_FIBULA_R',
    visual_reference = None,
    geometry = {'shape': 'box', 'length_cm': 40.0, 'width_cm': 1.5, 'thickness_cm': 1.5},
)
