
from importlib import import_module
from pathlib import Path
from typing import List

from ..base import BoneSpec

def load_bones() -> List[BoneSpec]:
    bones = []
    for file in Path(__file__).parent.glob('*.py'):
        if file.name == '__init__.py':
            continue
        module = import_module(f'skeleton.bones.{file.stem}')
        bones.append(module.bone)
    return bones
