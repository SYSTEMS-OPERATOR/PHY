from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Any

from skeleton.base import BoneSpec


@dataclass
class ValidatorAgent:
    """Validate a list of BoneSpec objects against a dataset."""

    skeleton: List[BoneSpec]
    dataset: Dict[str, Any]
    results: Dict[str, Any] = field(default_factory=dict)

    REQUIRED_KEYS = ["length_cm", "mass_g", "density_kg_m3"]
    MATERIALS = ["Ti6Al4V", "CFRP", "PEEK", "UHMWPE"]

    def run_all_checks(self) -> None:
        self.results = {
            "missing_metrics": [],
            "out_of_range": {},
            "material_fail": [],
            "summary": {"pass": True, "warnings": 0},
        }
        self._check_bone_count()
        self._check_metrics()
        self._check_volumes()
        self._check_material_tables()
        self._check_material_roundtrip()
        warnings = len(self.results["missing_metrics"]) + len(self.results["material_fail"])
        self.results["summary"] = {
            "pass": warnings < 10,
            "warnings": warnings,
        }

    def _check_bone_count(self) -> None:
        expected = 212  # 206 + ossicles
        actual = len(self.skeleton)
        self.results["bone_count"] = actual
        ok = actual >= expected
        self.results["bone_count_ok"] = ok

    def _check_metrics(self) -> None:
        missing = []
        for bone in self.skeleton:
            if bone.dataset_key is None:
                continue
            metrics = self.dataset.get(bone.dataset_key, {})
            for key in self.REQUIRED_KEYS:
                val = metrics.get(key)
                if val is None:
                    missing.append(bone.unique_id)
                    break
        self.results["missing_metrics"] = missing

    def _check_volumes(self) -> None:
        out = {}
        for bone in self.skeleton:
            if bone.dataset_key is None:
                continue
            metrics = self.dataset.get(bone.dataset_key, {})
            vol_geo = bone.volume_cm3()
            mass_g = metrics.get("mass_g")
            density = metrics.get("density_kg_m3")
            if None in (vol_geo, mass_g, density):
                continue
            derived = (mass_g / 1000.0) / density * 1e6
            if derived == 0:
                continue
            diff = abs(vol_geo - derived) / derived
            if diff > 0.07:
                out[bone.unique_id] = diff
        self.results["out_of_range"] = out

    def _check_material_tables(self) -> None:
        table = self.dataset.get("SyntheticMaterials", {})
        missing = [m for m in self.MATERIALS if m not in table]
        if missing:
            self.results["material_fail"].append({"missing_materials": missing})

    def _check_material_roundtrip(self) -> None:
        for bone in self.skeleton:
            if bone.dataset_key is None:
                continue
            vol = bone.volume_cm3()
            if vol is None:
                continue
            bone.set_embodiment("physical", {"density": bone.material.get("density", 1800)})
            bone.set_material("organic")
            base_density = bone.material.get("density")
            bone.set_material("Ti6Al4V")
            new_density = bone.material.get("density")
            bone.set_material("organic")
            if bone.material.get("density") != base_density or new_density == base_density:
                self.results["material_fail"].append(bone.unique_id)

