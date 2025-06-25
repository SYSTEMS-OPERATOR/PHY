import numpy as np
from skeleton.base import BoneSpec
from geometry.geometry_agent import GeometryAgent


def test_inertia_positive_definite():
    bone = BoneSpec(
        name='Test',
        bone_type='long',
        location={},
        articulations=[],
        dimensions={'length_cm': 10.0, 'width_cm': 2.0, 'thickness_cm': 2.0},
        function=[],
        notable_features=[],
        developmental_notes='',
        variations='',
        unique_id='TEST',
    )
    bone.set_material('organic')
    bone.set_embodiment('physical', bone.material)
    GeometryAgent(bone).compute()
    I = np.array(bone.geometry['inertia_kgm2'])
    eig = np.linalg.eigvalsh(I)
    assert np.all(eig > 0)

