
"""Base classes for bone agents.

This module defines :class:`BoneSpec`, the lightweight container used by all
bone modules. The class now includes an ``embodiment`` attribute and helpers to
manage context-aware material data as outlined in ``AGENTS.md``.  Allowed
embodiment states are ``"virtual"``, ``"digital"``, ``"metaphysical"``, and
``"physical"`` (or any user-defined extension).
"""

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
    embodiment: str = "virtual"
    material_attributes: Optional[Dict[str, float]] = None

    def __post_init__(self) -> None:
        """Initialize dynamic docstring for this bone object."""
        self.__doc__ = f"{self.name} bone agent (default embodiment: {self.embodiment})."

    def mass_kg(self) -> Optional[float]:
        if self.embodiment != "physical":
            raise ValueError(
                f"{self.name} is not embodied physically—mass undefined in state '{self.embodiment}'."
            )
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

    def set_embodiment(self, state: str, material: Optional[Dict[str, float]] = None) -> None:
        """Update the embodiment state and material context."""
        self.embodiment = state
        if state == "physical":
            if material is not None:
                self.material.update(material)
            self.material_attributes = self.material.copy()
        else:
            self.material_attributes = None

    def self_state(self) -> Dict[str, object]:
        """Return a dict summarizing the bone's current state."""
        return {
            "bone": self.name,
            "uid": self.unique_id,
            "embodiment": self.embodiment,
            "material": self.material.get("name") if self.material_attributes else "non-tangible",
            "material_attributes": self.material_attributes if self.material_attributes else "N/A",
            "context": "simulated" if self.embodiment in ["virtual", "digital"] else "manifested",
            "module": __file__,
        }
