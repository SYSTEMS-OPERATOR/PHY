
from dataclasses import dataclass, field
from typing import Dict, List, Optional

@dataclass
class BoneSpec:
    name: str
    bone_type: str
    location: Dict[str, str]
    articulations: List[Dict[str, str]]
    dimensions: Dict[str, Optional[float]]
    function: List[str]
    notable_features: List[str]
    developmental_notes: str
    variations: str
    unique_id: str
    visual_reference: Optional[str] = None
    material: Dict[str, float] = field(default_factory=lambda: {"name": "bone", "density": 1800.0})
    geometry: Dict[str, float] = field(default_factory=dict)

    def mass_kg(self) -> Optional[float]:
        vol = self.volume_cm3()
        if vol is None:
            return None
        return self.material["density"] * vol / 1e6

    def volume_cm3(self) -> Optional[float]:
        l = self.dimensions.get("length_cm")
        w = self.dimensions.get("width_cm")
        t = self.dimensions.get("thickness_cm")
        if None not in (l, w, t):
            return l * w * t
        return None

    def set_material(self, name: str, density: float) -> None:
        self.material["name"] = name
        self.material["density"] = density
