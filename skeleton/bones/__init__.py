
from importlib import import_module
from pathlib import Path
from typing import List

from ..base import BoneSpec
from ..field import SkeletonField
from ..datasets import load_dataset

def load_bones(dataset_name: str = "female_21_baseline") -> List[BoneSpec]:
    bones: List[BoneSpec] = []
    for file in Path(__file__).parent.glob('*.py'):
        if file.name == '__init__.py':
            continue
        module = import_module(f'skeleton.bones.{file.stem}')
        bones.append(module.bone)

    dataset = load_dataset(dataset_name)
    for b in bones:
        # keep dataset reference for later material lookups
        b.dataset = dataset
        b.apply_dataset(dataset)

    # establish entanglements based on articulations
    name_map = {b.name: b for b in bones}
    for bone in bones:
        for art in bone.articulations:
            other = name_map.get(art.get('bone'))
            if other and other is not bone:
                bone.entangle(other)

    return bones


def load_field(dataset_name: str = "female_21_baseline") -> SkeletonField:
    """Return a SkeletonField with all discovered bones registered."""
    bones = load_bones(dataset_name)
    field = SkeletonField(bones)
    return field
