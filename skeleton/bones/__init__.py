
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from typing import List

from ..base import BoneSpec
from ..field import SkeletonField
from ..datasets import load_dataset

def load_bones(dataset_name: str = "female_21_baseline") -> List[BoneSpec]:
    """Construct independent bones from source, in filename order.

    Execute definitions in fresh namespaces rather than using (or reloading)
    the mutable singleton exposed by each legacy module's wrapper functions.
    """
    dataset = load_dataset(dataset_name)
    bones: List[BoneSpec] = []
    for file in sorted(Path(__file__).parent.glob('*.py')):
        if file.name == '__init__.py':
            continue
        spec = spec_from_file_location(f'{__name__}.{file.stem}', file)
        if spec is None or spec.loader is None:
            raise ImportError(f'Cannot load bone definition: {file}')
        module = module_from_spec(spec)
        spec.loader.exec_module(module)
        bone = module.bone
        bone.apply_dataset(dataset)
        bones.append(bone)

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
