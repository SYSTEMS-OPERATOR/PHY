from ..base import BoneSpec

bone = BoneSpec(
    name = 'Clavicle',
    bone_type = 'long',
    location = {'region': 'shoulder', 'proximal_connection': 'sternum', 'distal_connection': 'scapula'},
    articulations = [],
    dimensions = {'length_cm': 14.0, 'width_cm': 1.0, 'thickness_cm': 1.0},
    function = ['strut for shoulder'],
    notable_features = [],
    developmental_notes = 'ossifies first among long bones',
    variations = '',
    unique_id = 'BONE_CLAVICLE_L',
    visual_reference = None,
    geometry = {'shape': 'box', 'length_cm': 14.0, 'width_cm': 1.0, 'thickness_cm': 1.0},
)
