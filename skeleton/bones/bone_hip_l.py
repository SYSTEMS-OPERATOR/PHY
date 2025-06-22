from ..base import BoneSpec

bone = BoneSpec(
    name = 'Hip Bone',
    bone_type = 'irregular',
    location = {'region': 'pelvis', 'proximal_connection': 'sacrum', 'distal_connection': 'femur'},
    articulations = [],
    dimensions = {'length_cm': None, 'width_cm': None, 'thickness_cm': None},
    function = ['supports trunk'],
    notable_features = [],
    developmental_notes = 'ilium, ischium, pubis fuse in adolescence',
    variations = '',
    unique_id = 'BONE_HIP_L',
    visual_reference = None,
    geometry = {},
)
