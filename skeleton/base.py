
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

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple


class MarrowAgent:
    """Bioelectric regulator for a bone domain."""

    def __init__(self, voltage: float = 0.0, charge: float = 0.0, energy: float = 1.0) -> None:
        self.voltage = voltage
        self.charge = charge
        self.energy = energy

    def regulate(self, voltage_in: float, signal_type: str = "EMG") -> float:
        """Return regulated voltage while updating charge and energy."""
        self.voltage = 0.5 * (self.voltage + voltage_in)
        if signal_type in {"EMG", "EKG"}:
            self.charge += self.voltage * 0.01
        else:
            self.charge += self.voltage * 0.005
        self.energy += self.voltage * 0.01
        return self.voltage

    def audit(self) -> Dict[str, float]:
        return {"voltage": self.voltage, "charge": self.charge, "energy": self.energy}



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
    marrow: MarrowAgent = field(default_factory=MarrowAgent)
    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    orientation: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    load: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    torsion: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    state_faults: List[str] = field(default_factory=list)
    resonance: List[str] = field(default_factory=list)

    def __post_init__(self) -> None:
        """Initialize dynamic docstring and default network state."""
        self.__doc__ = (
            f"{self.name} bone agent (default embodiment: {self.embodiment})."
            " Supports metaphysical entanglement."
        )
        if not self.domain_id:
            self.domain_id = self.unique_id
        self.marrow = MarrowAgent()
        self.state_faults = []
        self.resonance = []


    def mass_kg(self) -> Optional[float]:
        try:
            if self.embodiment != "physical":
                raise ValueError(
                    f"{self.name} is not embodied physically—mass undefined in state '{self.embodiment}'."
                )
            vol = self.volume_cm3()
            if vol is None:
                return None
            return self.material["density"] * vol / 1e6
        except Exception as e:
            self.state_faults.append(f"Mass error: {e}")
            return None

    def volume_cm3(self) -> Optional[float]:
        try:
            l = self.dimensions.get("length_cm")
            w = self.dimensions.get("width_cm")
            t = self.dimensions.get("thickness_cm")
            if None not in (l, w, t):
                return l * w * t
            return None
        except Exception as e:
            self.state_faults.append(f"Volume error: {e}")
            return None

    def set_material(self, name: str, density: float) -> None:
        try:
            self.material["name"] = name
            self.material["density"] = float(density)
        except Exception as e:
            self.state_faults.append(f"Material error: {e}")

    def set_embodiment(self, state: str, material: Optional[Dict[str, float]] = None) -> None:
        """Update the embodiment state and material context."""
        try:
            self.embodiment = state
            if state == "physical":
                if material is not None:
                    self.material.update(material)
                self.material_attributes = self.material.copy()
            else:
                self.material_attributes = None
        except Exception as e:
            self.state_faults.append(f"Embodiment error: {e}")

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
        return self.current_state()

    def current_state(self) -> Dict[str, object]:
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
            "voltage": self.voltage_potential,
            "links": list(self.entanglement_links),
            "position": self.position,
            "orientation": self.orientation,
            "load": self.load,
            "torsion": self.torsion,
            "faults": list(self.state_faults),
        }

    def update_state(
        self,
        position: Optional[Tuple[float, float, float]] = None,
        orientation: Optional[Tuple[float, float, float, float]] = None,
        load: Optional[Tuple[float, float, float]] = None,
        torsion: Optional[Tuple[float, float, float]] = None,
    ) -> None:
        try:
            if position is not None:
                self.position = position
            if orientation is not None:
                self.orientation = orientation
            if load is not None:
                self.load = load
            if torsion is not None:
                self.torsion = torsion
        except Exception as e:
            self.state_faults.append(f"Update error: {e}")

    def is_healthy(self) -> bool:
        return len(self.state_faults) == 0

    def clear_faults(self) -> None:
        self.state_faults = []

    def report_faults(self) -> List[str]:
        return list(self.state_faults)

    def notify_fault(self, field: Optional["SkeletonField"] = None) -> None:
        """Notify entangled bones of the most recent fault."""
        message = self.state_faults[-1] if self.state_faults else "fault"
        for link_id in self.entanglement_links:
            try:
                if field:
                    other = field.bones.get(link_id)
                else:
                    other = None
                if other is not None:
                    other.receive_fault_notice(self.domain_id, message)
            except Exception as e:
                self.state_faults.append(f"Notify fault error to {link_id}: {e}")

    def receive_fault_notice(self, from_domain: str, message: str) -> None:
        try:
            self.state_faults.append(f"Neighbor {from_domain} fault: {message}")
        except Exception as e:
            self.state_faults.append(f"Receive fault notice error from {from_domain}: {e}")

    def entangle(self, other_bone: "BoneSpec") -> None:
        """Create a bidirectional entanglement link with another bone."""
        try:
            if other_bone.domain_id not in self.entanglement_links:
                self.entanglement_links.append(other_bone.domain_id)
            if self.domain_id not in other_bone.entanglement_links:
                other_bone.entanglement_links.append(self.domain_id)
        except Exception as e:
            self.state_faults.append(f"Entangle error: {e}")

    def receive_signal_packet(self, signal: Dict[str, float], from_domain: str) -> float:
        """Accept a structured signal from another domain and regulate voltage."""
        try:
            voltage = signal.get("voltage", 0.0)
            stype = signal.get("type", "EMG")
            self.voltage_potential = self.marrow.regulate(voltage, stype)
            return self.voltage_potential
        except Exception as e:
            self.state_faults.append(f"Receive error from {from_domain}: {e}")
            return self.voltage_potential

    def emit_signal_packet(self, to_bone: "BoneSpec", signal: Dict[str, float]) -> float:
        """Emit a structured signal to another entangled bone."""
        try:
            if to_bone.domain_id not in self.entanglement_links:
                raise ValueError(f"{to_bone.domain_id} not entangled with {self.domain_id}")
            return to_bone.receive_signal_packet(signal, self.domain_id)
        except Exception as e:
            self.state_faults.append(f"Signal error: {e}")
            return 0.0

    def transcend(self, intent: Optional[str] = None) -> None:
        """Increase internal energy and optionally log metaphysical intent."""
        try:
            self.marrow.energy += 1.0
            if intent:
                self.resonance.append(intent)
        except Exception as e:
            self.state_faults.append(f"Transcend error: {e}")

    def invoke(self, field: "SkeletonField", intent: str) -> None:
        """Propagate a symbolic signal through the field."""
        try:
            signal = {"voltage": self.voltage_potential, "type": intent}
            field.propagate(self.domain_id, signal)
        except Exception as e:
            self.state_faults.append(f"Invoke error: {e}")

    def audit(self) -> Dict[str, object]:
        """Return full state information including marrow audit."""
        state = self.current_state()
        state["marrow"] = self.marrow.audit()
        state["resonance"] = list(self.resonance)
        return state

