"""Datasets of bone parameters for various embodiments."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, List

from ..base import BoneSpec


DATA_DIR = Path(__file__).parent


def load_dataset(name: str) -> Dict[str, dict]:
    """Return raw dataset dictionary by name.

    Parameters
    ----------
    name: str
        Dataset filename without extension.
    """
    file = DATA_DIR / f"{name}.json"
    with open(file, "r", encoding="utf-8") as fh:
        return json.load(fh)


def bones_from_dataset(name: str) -> List[BoneSpec]:
    """Construct BoneSpec objects for major bones from a dataset."""
    data = load_dataset(name)
    bones: List[BoneSpec] = []
    material_defaults = data.get("BoneMaterialProperties", {})
    for bone_name, props in data.items():
        if bone_name in {"BoneMaterialProperties", "SyntheticMaterials"}:
            continue
        dims = {}
        for k in ("length_cm", "width_cm", "thickness_cm"):
            if k in props:
                dims[k] = props[k]
        if "height_cm" in props:
            dims["height_cm"] = props["height_cm"]
        bones.append(
            BoneSpec(
                name=bone_name,
                bone_type="generic",
                location={},
                articulations=[],
                dimensions=dims,
                function=[],
                notable_features=[],
                developmental_notes="",
                variations="",
                unique_id=f"DATA_{bone_name.upper()}",
                material={"name": "bone", "density": props.get("density_kg_m3", 1800)},
                geometry={},
            )
        )
    return bones
