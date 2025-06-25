import os
import sys
import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from skeleton.datasets import load_dataset, bones_from_dataset
from skeleton.field import SkeletonField

@pytest.fixture(scope="session")
def dataset():
    return load_dataset("female_21_baseline")

@pytest.fixture(scope="session")
def skeleton_field(dataset):
    all_bones = bones_from_dataset("female_21_baseline")
    bones = []
    for b in all_bones:
        m = dataset.get(b.name, {})
        if all(k in m for k in ("length_cm", "width_cm", "thickness_cm", "mass_g", "density_kg_m3")):
            bones.append(b)
    field = SkeletonField(bones)
    for b in bones:
        b.set_material("organic")
        b.set_embodiment("physical", b.material)
    return field
