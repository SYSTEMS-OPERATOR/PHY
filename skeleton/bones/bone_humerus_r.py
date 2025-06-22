from ..base import BoneSpec

bone = BoneSpec(
    name = 'Humerus',
    bone_type = 'long',
    location = {'region': 'upper arm', 'proximal_connection': 'scapula', 'distal_connection': 'radius/ulna'},
    articulations = [],
    dimensions = {'length_cm': 35.0, 'width_cm': 2.5, 'thickness_cm': 2.5},
    function = ['lever for arm'],
    notable_features = ['deltoid tuberosity'],
    developmental_notes = 'growth plates at both ends',
    variations = '',
    unique_id = 'BONE_HUMERUS_R',
    visual_reference = None,
    geometry = {'shape': 'box', 'length_cm': 35.0, 'width_cm': 2.5, 'thickness_cm': 2.5},
)
