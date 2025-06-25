from export.urdf_exporter import bones_to_urdf
from joints.joint_spec import hinge
from skeleton.datasets import bones_from_dataset


def test_urdf_generation():
    bones = bones_from_dataset('female_21_baseline')[:2]
    j = hinge('j', bones[0].unique_id, bones[1].unique_id)
    urdf = bones_to_urdf(bones[0], [j], bones)
    assert '<robot' in urdf
    assert bones[0].unique_id in urdf
