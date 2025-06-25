from geometry.geometry_agent import GeometryAgent
from skeleton.datasets import bones_from_dataset
import numpy as np


def test_inertia_positive_definite():
    bone = bones_from_dataset("female_21_baseline")[0]
    bone.set_material("organic")
    bone.set_embodiment("physical", bone.material)
    GeometryAgent(bone).compute()
    inertia = np.array(bone.geometry["inertia_kgm2"])
    eig = np.linalg.eigvals(inertia)
    assert np.all(eig > 0)
