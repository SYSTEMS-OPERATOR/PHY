
from importlib import import_module
from pathlib import Path
from typing import List

from ..base import BoneSpec

def load_bones() -> List[BoneSpec]:
    bones: List[BoneSpec] = []
    for file in Path(__file__).parent.glob('*.py'):
        if file.name == '__init__.py':
            continue
        module = import_module(f'skeleton.bones.{file.stem}')
        bones.append(module.bone)

    # establish entanglements based on articulations
    name_map = {b.name: b for b in bones}
    for bone in bones:
        for art in bone.articulations:
            other = name_map.get(art.get('bone'))
            if other and other is not bone:
                bone.entangle(other)

    return bones
