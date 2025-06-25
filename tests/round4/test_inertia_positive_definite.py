from geometry.geometry_agent import GeometryAgent
from skeleton.datasets import bones_from_dataset


def test_inertia_positive_definite():
    bones = bones_from_dataset('female_21_baseline')[:1]
    agent = GeometryAgent()
    data = agent.build_geometry(bones[0])
    assert all(i > 0 for i in data.inertia_kgm2)
