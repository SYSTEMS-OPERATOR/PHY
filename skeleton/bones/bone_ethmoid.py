from ..base import BoneSpec

bone = BoneSpec(
    name = 'Ethmoid',
    bone_type = 'irregular',
    location = {'region': 'skull', 'proximal_connection': 'frontal', 'distal_connection': 'vomer'},
    articulations = [{'bone': 'vomer', 'joint_type': 'suture'}],
    dimensions = {'length_cm': 5.0, 'width_cm': 5.0, 'thickness_cm': 0.4},
    function = ['supports olfaction'],
    notable_features = ['crista galli'],
    developmental_notes = 'cartilaginous origin',
    variations = '',
    unique_id = 'BONE_ETHMOID',
    visual_reference = None,
    geometry = {'shape': 'box', 'length_cm': 5.0, 'width_cm': 5.0, 'thickness_cm': 0.4},
)
