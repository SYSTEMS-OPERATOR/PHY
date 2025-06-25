from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional

from skeleton.field import SkeletonField
from skeleton.base import BoneSpec


@dataclass
class ValidatorAgent:
    """Validate skeleton metrics against a dataset."""

    skeleton: SkeletonField
    dataset: Dict[str, dict]
    results: Dict[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.results = {
            "missing_metrics": [],
            "out_of_range": {},
            "material_fail": [],
            "summary": {"pass": False, "warnings": 0},
        }

    def check_bone_count(self) -> bool:
        expected = 206
        actual = len(self.skeleton.bones)
        if actual != expected:
            self.results["summary"]["warnings"] += 1
            self.results["bone_count"] = actual
            return False
        return True

    def check_required_metrics(self) -> bool:
        required = ["length_cm", "mass_g", "density_kg_m3"]
        for bone in self.skeleton.bones.values():
            metrics = self.dataset.get(bone.dataset_key or bone.name, {})
            for key in required:
                if metrics.get(key) is None:
                    self.results["missing_metrics"].append((bone.unique_id, key))
        if self.results["missing_metrics"]:
            self.results["summary"]["warnings"] += len(self.results["missing_metrics"])
            return False
        return True

    def check_volume_consistency(self) -> bool:
        for bone in self.skeleton.bones.values():
            metrics = self.dataset.get(bone.dataset_key or bone.name, {})
            mass = metrics.get("mass_g")
            density = metrics.get("density_kg_m3")
            if mass is None or density is None:
                continue
            derived_vol = mass * 1000 / density  # cm^3
            geom_vol = None
            l = metrics.get("length_cm")
            w = metrics.get("width_cm")
            t = metrics.get("thickness_cm")
            if None not in (l, w, t):
                geom_vol = l * w * t
            if geom_vol is not None:
                diff = abs(geom_vol - derived_vol) / geom_vol
                if diff > 0.07:
                    self.results.setdefault("out_of_range", {})[bone.unique_id] = diff
        if self.results.get("out_of_range"):
            self.results["summary"]["warnings"] += len(self.results["out_of_range"])
        return len(self.results.get("out_of_range", {})) == 0

    def check_material_tables(self) -> bool:
        table = self.dataset.get("SyntheticMaterials", {})
        required = {"Ti6Al4V", "CFRP", "PEEK", "UHMWPE"}
        missing = required - set(table)
        if missing:
            self.results["material_fail"].extend(sorted(missing))
            self.results["summary"]["warnings"] += len(missing)
            return False
        return True

    def check_material_switch(self) -> bool:
        failed = False
        for bone in self.skeleton.bones.values():
            orig_density = bone.material.get("density")
            bone.set_material("Ti6Al4V")
            ti_density = bone.material.get("density")
            bone.set_material("organic")
            if bone.material.get("density") != orig_density or ti_density is None:
                self.results["material_fail"].append(bone.unique_id)
                failed = True
        if failed:
            self.results["summary"]["warnings"] += len(self.results["material_fail"])
        return not failed

    def run_all_checks(self) -> Dict[str, object]:
        self.check_bone_count()
        self.check_required_metrics()
        self.check_volume_consistency()
        self.check_material_tables()
        self.check_material_switch()
        self.results["summary"]["pass"] = (
            not self.results["missing_metrics"]
            and not self.results["material_fail"]
        )
        return self.results
