
"""Base classes for bone agents.

This module defines :class:`BoneSpec`, the lightweight container used by all
bone modules. The class now includes an ``embodiment`` attribute and helpers to
manage context-aware material data as outlined in ``AGENTS.md``.  Allowed
embodiment states are ``"virtual"``, ``"digital"``, ``"metaphysical"``, and
``"physical"`` (or any user-defined extension).

The module also introduces :class:`MarrowAgent` and entanglement helpers so
bones can participate in a metaphysical bionic network, following the
"Metaphysical Bionic Entanglement Protocol" directive."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional


class MarrowAgent:
    """Bioelectric regulator for a bone."""

    def __init__(self, voltage: float = 0.0, charge: float = 0.0) -> None:
        self.voltage = voltage
        self.charge = charge

    def regulate(self, voltage_in: float, signal_type: str = "EMG") -> float:
        """Regulate incoming voltage and update internal state."""
        self.voltage = (self.voltage + voltage_in) / 2.0
        self.charge += self.voltage * 0.1
        return self.voltage

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
    domain_id: str = ""
    ground_state: bool = False
    voltage_potential: float = 0.0
    entanglement_links: List["BoneSpec"] = field(default_factory=list)
    marrow: MarrowAgent = field(init=False)

    def __post_init__(self) -> None:
        """Initialize dynamic docstring and default network state."""
        self.__doc__ = (
            f"{self.name} bone agent (default embodiment: {self.embodiment})."
            " Supports metaphysical entanglement."
        )
        if not self.domain_id:
            self.domain_id = self.unique_id
        self.marrow = MarrowAgent()

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

    def entangle(self, other_bone: "BoneSpec") -> None:
        """Create a bidirectional entanglement link."""
        if other_bone not in self.entanglement_links:
            self.entanglement_links.append(other_bone)
        if self not in other_bone.entanglement_links:
            other_bone.entanglement_links.append(self)

    def receive_signal(self, voltage: float, signal_type: str = "EMG", from_domain: Optional[str] = None) -> float:
        """Receive a signal and update voltage potential via the marrow agent."""
        regulated = self.marrow.regulate(voltage, signal_type)
        self.voltage_potential = regulated
        return regulated

    def emit_signal(self, to_bone: Optional["BoneSpec"], voltage: float, signal_type: str = "EMG") -> None:
        """Emit a signal to another bone or broadcast to entangled links."""
        regulated = self.marrow.regulate(voltage, signal_type)
        self.voltage_potential = regulated
        targets = self.entanglement_links if to_bone is None else [to_bone]
        for target in targets:
            target.receive_signal(regulated, signal_type, from_domain=self.domain_id)

    def self_state(self) -> Dict[str, object]:
        """Return a dict summarizing the bone's current state."""
        return {
            "bone": self.name,
            "uid": self.unique_id,
            "domain_id": self.domain_id,
            "embodiment": self.embodiment,
            "material": self.material.get("name") if self.material_attributes else "non-tangible",
            "material_attributes": self.material_attributes if self.material_attributes else "N/A",
            "voltage": self.voltage_potential,
            "entanglements": [b.domain_id for b in self.entanglement_links],
            "context": "simulated" if self.embodiment in ["virtual", "digital"] else "manifested",
            "module": __file__,
        }
