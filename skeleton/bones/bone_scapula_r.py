from ..base import BoneSpec

bone = BoneSpec(
    name = 'Scapula',
    bone_type = 'flat',
    location = {'region': 'shoulder', 'proximal_connection': 'clavicle', 'distal_connection': 'humerus'},
    articulations = [],
    dimensions = {'length_cm': None, 'width_cm': None, 'thickness_cm': None},
    function = ['attachment for arm'],
    notable_features = ['glenoid fossa'],
    developmental_notes = 'ossifies from multiple centers',
    variations = '',
    unique_id = 'BONE_SCAPULA_R',
    visual_reference = None,
    geometry = {},
)
