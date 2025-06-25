from geometry.geometry_agent import GeometryAgent
from skeleton.datasets import bones_from_dataset, load_dataset


def test_inertia_positive_definite():
    bones = bones_from_dataset('female_21_baseline')
    dataset = load_dataset('female_21_baseline')
    femur = next(b for b in bones if b.unique_id == 'DATA_FEMUR')
    g = GeometryAgent(femur, dataset)
    g.compute()
    inertia = g.bone.geometry['inertia_kgm2']
    assert all(inertia[i][i] > 0 for i in range(3))
