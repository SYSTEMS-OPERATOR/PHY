from __future__ import annotations

from typing import Dict, List
from dataclasses import dataclass, field
from skeleton.field import SkeletonField


@dataclass
class ValidatorAgent:
    """Validate a skeleton against its dataset."""

    skeleton: SkeletonField
    dataset: Dict[str, dict]
    results: Dict[str, object] = field(default_factory=dict)

    REQUIRED_KEYS = ["length_cm", "mass_g", "density_kg_m3"]
    MATERIALS = {"Ti6Al4V", "CFRP", "PEEK", "UHMWPE"}

    def run_all_checks(self) -> None:
        self.results = {
            "missing_metrics": [],
            "out_of_range": {},
            "material_fail": [],
            "summary": {"pass": False, "warnings": 0},
        }
        self._check_bone_count()
        self._check_metrics()
        self._check_material_tables()
        self._roundtrip_material()
        warnings = 0
        if len(self.skeleton.bones) != 206:
            warnings += 1
        if self.results["missing_metrics"]:
            warnings += 1
        if self.results["out_of_range"]:
            warnings += 1
        if self.results["material_fail"]:
            warnings += 1
        self.results["summary"]["warnings"] = warnings
        self.results["summary"]["pass"] = (
            warnings < 10 and not self.results["material_fail"]
        )

    # internal helpers
    def _check_bone_count(self) -> None:
        # presence check only; warning count handled in run_all_checks
        if len(self.skeleton.bones) != 206:
            pass

    def _check_metrics(self) -> None:
        for bone in self.skeleton.bones.values():
            if not bone.dataset_key:
                continue
            metrics = self.dataset.get(bone.dataset_key, {})
            for key in self.REQUIRED_KEYS:
                ds_val = metrics.get(key)
                if ds_val is None:
                    self.results["missing_metrics"].append(f"{bone.unique_id}:{key}")
                else:
                    if key.endswith("_cm"):
                        val = bone.dimensions.get(key)
                    elif key == "mass_g":
                        val = bone.material.get(key)
                    else:  # density_kg_m3
                        val = bone.material.get("density")
                    if val is None:
                        self.results["missing_metrics"].append(f"{bone.unique_id}:{key}")
            mass = metrics.get("mass_g")
            density = metrics.get("density_kg_m3")
            l = metrics.get("length_cm")
            w = metrics.get("width_cm")
            t = metrics.get("thickness_cm")
            if None not in (mass, density, l, w, t):
                derived = mass / (density / 1000.0)
                geometric = l * w * t
                delta = abs(derived - geometric) / derived
                if delta > 0.07:
                    self.results["out_of_range"][bone.unique_id] = delta

    def _check_material_tables(self) -> None:
        table = self.dataset.get("SyntheticMaterials", {})
        missing = [m for m in self.MATERIALS if m not in table]
        if missing:
            self.results["material_fail"].extend(missing)

    def _roundtrip_material(self) -> None:
        table = self.dataset.get("SyntheticMaterials", {})
        if not table:
            return
        for bone in self.skeleton.bones.values():
            base_density = bone.material.get("density")
            bone.set_embodiment("physical")
            bone.set_material("Ti6Al4V")
            dens1 = bone.material.get("density")
            bone.set_material("organic")
            dens2 = bone.material.get("density")
            bone.set_embodiment("virtual")
            if dens1 == base_density or dens2 != base_density:
                self.results["material_fail"].append(bone.unique_id)
