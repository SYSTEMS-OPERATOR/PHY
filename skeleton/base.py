"""Canonical bone data model for fabrication and simulation.

`BoneSpec` is the single source of truth for bone-level records in PHY.
It keeps fabrication-critical fields explicit while preserving compatibility
with existing per-bone modules under ``skeleton/bones``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any
import warnings


CANONICAL_UNITS = {
    "length": "mm",
    "mass": "kg",
    "density": "kg/m^3",
    "inertia": "kg*m^2",
}


@dataclass
class BoneSpec:
    # Canonical identity
    name: str
    bone_type: str
    location: Dict[str, str]
    articulations: List[Dict[str, str]]
    dimensions: Dict[str, Optional[float]]

    # Legacy descriptive fields retained for compatibility
    function: List[str]
    notable_features: List[str]
    developmental_notes: str
    variations: str
    unique_id: str

    # Canonical fabrication fields (required by policy; defaults supplied)
    latin_name: Optional[str] = None
    region: str = ""
    units: Dict[str, str] = field(default_factory=lambda: dict(CANONICAL_UNITS))
    geometry: Dict[str, Any] = field(default_factory=dict)
    material: Dict[str, float] = field(default_factory=lambda: {"name": "bone", "density": 1800.0})
    physics: Dict[str, float] = field(default_factory=dict)
    connections: Dict[str, Any] = field(default_factory=dict)
    joint_interfaces: List[Dict[str, Any]] = field(default_factory=list)
    mount_points: List[Dict[str, Any]] = field(default_factory=list)
    manufacturing_notes: List[str] = field(default_factory=list)
    tolerance: Dict[str, float] = field(default_factory=lambda: {"default_mm": 0.5})
    references: List[str] = field(default_factory=list)
    revision: str = "1.0.0"
    source_ids: List[str] = field(default_factory=list)

    # Existing optional/operational fields
    visual_reference: Optional[str] = None
    embodiment: str = "virtual"
    material_attributes: Optional[Dict[str, float]] = None
    domain_id: str = ""
    ground_state: bool = False
    voltage_potential: float = 0.0
    entanglement_links: List[str] = field(default_factory=list)
    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    orientation: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    load: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    torsion: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    state_faults: List[str] = field(default_factory=list)
    resonance: List[str] = field(default_factory=list)
    dataset: Optional[Dict[str, dict]] = None
    dataset_key: Optional[str] = None
    metric_sources: Dict[str, str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.domain_id:
            self.domain_id = self.unique_id
        if not self.region:
            self.region = self.location.get("region", "unspecified")
        if not self.connections:
            self.connections = {
                "parent": self.location.get("proximal_connection"),
                "children": [a.get("bone") for a in self.articulations if a.get("bone")],
            }
        if not self.source_ids:
            self.source_ids = [self.unique_id]
        if self.dataset is not None:
            self.apply_dataset(self.dataset)

    def _dimensions_mm(self) -> Dict[str, Optional[float]]:
        """Normalize cm-based dimensions to canonical mm."""
        mapped: Dict[str, Optional[float]] = {}
        for key, val in self.dimensions.items():
            if val is None:
                mapped[key] = None
                continue
            if key.endswith("_cm"):
                mapped[key.replace("_cm", "_mm")] = float(val) * 10.0
            elif key.endswith("_mm"):
                mapped[key] = float(val)
            else:
                mapped[key] = float(val)
        return mapped

    def _volume_m3(self) -> Optional[float]:
        dmm = self._dimensions_mm()
        l = dmm.get("length_mm")
        w = dmm.get("width_mm")
        t = dmm.get("thickness_mm")
        if None in (l, w, t):
            return None
        # mm^3 -> m^3
        return (l * w * t) / 1e9

    def mass_kg(self) -> Optional[float]:
        if self.embodiment != "physical":
            self.state_faults.append(
                f"Mass error: {self.name} is not embodied physically in state '{self.embodiment}'"
            )
            return None
        vol = self._volume_m3()
        if vol is None:
            return None
        density = float(self.material.get("density", 1800.0))
        return density * vol

    def set_material(self, material_key: Any) -> None:
        """Update material properties from dataset table or mapping."""
        if isinstance(material_key, dict):
            self.material.update(material_key)
            return

        if self.dataset is None:
            self.state_faults.append("No dataset available for material lookup")
            return
        table = self.dataset.get("SyntheticMaterials", {})
        if material_key == "organic":
            props = self.dataset.get("BoneMaterialProperties", {})
            if not props:
                self.state_faults.append("Organic material properties missing")
                return
            self.material = {
                "name": "organic",
                "density": props.get("cortical_density_g_cm3", 1.8) * 1000,
                "Youngs_modulus_GPa": props.get("cortical_Youngs_modulus_GPa"),
                "tensile_strength_MPa": props.get("cortical_tensile_strength_MPa"),
                "compressive_strength_MPa": props.get("cortical_compressive_strength_MPa"),
            }
        elif material_key in table:
            mat = table[material_key]
            self.material = {"name": str(material_key)}
            self.material.update(mat)
            if "density_g_cm3" in mat:
                self.material["density"] = mat["density_g_cm3"] * 1000
        else:
            self.state_faults.append(f"Material {material_key} not found")

    def set_embodiment(self, state: str, material: Optional[Dict[str, float]] = None) -> None:
        """Compatibility setter used by existing scripts."""
        self.embodiment = state
        if material is not None:
            self.material.update(material)
        self.material_attributes = self.material.copy() if state == "physical" else None

    def get_material_properties(self) -> Dict[str, float]:
        return dict(self.material)

    def receive_signal(self, voltage: float, signal_type: str = "EMG", from_domain: Optional[str] = None) -> float:
        self.voltage_potential = 0.5 * (self.voltage_potential + voltage)
        return self.voltage_potential

    def emit_signal_packet(self, to_bone: "BoneSpec", signal: Dict[str, float]) -> float:
        if to_bone.domain_id not in self.entanglement_links:
            self.state_faults.append(f"Signal error: {to_bone.domain_id} not linked")
            return 0.0
        return to_bone.receive_signal_packet(signal, self.domain_id)

    def receive_signal_packet(self, signal: Dict[str, float], from_domain: str) -> float:
        return self.receive_signal(signal.get("voltage", 0.0), signal.get("type", "EMG"), from_domain)

    def entangle(self, other_bone: "BoneSpec") -> None:
        if other_bone.domain_id not in self.entanglement_links:
            self.entanglement_links.append(other_bone.domain_id)
        if self.domain_id not in other_bone.entanglement_links:
            other_bone.entanglement_links.append(self.domain_id)

    def transcend(self, intent: Optional[str] = None) -> None:
        """Compatibility symbolic API (non-fabrication-critical)."""
        if intent:
            self.resonance.append(intent)

    def invoke(self, field: "SkeletonField", intent: str) -> None:
        signal = {"voltage": self.voltage_potential, "type": intent}
        field.propagate(self.domain_id, signal)

    def update_state(
        self,
        position: Optional[Tuple[float, float, float]] = None,
        orientation: Optional[Tuple[float, float, float, float]] = None,
        load: Optional[Tuple[float, float, float]] = None,
        torsion: Optional[Tuple[float, float, float]] = None,
    ) -> None:
        if position is not None:
            self.position = position
        if orientation is not None:
            self.orientation = orientation
        if load is not None:
            self.load = load
        if torsion is not None:
            self.torsion = torsion

    def current_state(self) -> Dict[str, object]:
        return {
            "bone": self.name,
            "uid": self.unique_id,
            "domain_id": self.domain_id,
            "embodiment": self.embodiment,
            "material": self.material.get("name", "bone"),
            "material_attributes": self.material_attributes if self.material_attributes else "N/A",
            "voltage": self.voltage_potential,
            "links": list(self.entanglement_links),
            "position": self.position,
            "orientation": self.orientation,
            "load": self.load,
            "torsion": self.torsion,
            "faults": list(self.state_faults),
        }

    def self_state(self) -> Dict[str, object]:
        return self.current_state()

    def is_healthy(self) -> bool:
        return len(self.state_faults) == 0

    def clear_faults(self) -> None:
        self.state_faults = []

    def report_faults(self) -> List[str]:
        return list(self.state_faults)

    def notify_fault(self, field: Optional["SkeletonField"] = None) -> None:
        message = self.state_faults[-1] if self.state_faults else "fault"
        for link_id in self.entanglement_links:
            other = field.bones.get(link_id) if field else None
            if other is not None:
                other.receive_fault_notice(self.domain_id, message)

    def receive_fault_notice(self, from_domain: str, message: str) -> None:
        self.state_faults.append(f"Neighbor {from_domain} fault: {message}")

    def audit(self) -> Dict[str, object]:
        return self.current_state()

    def apply_dataset(self, dataset: Dict[str, dict]) -> None:
        self.dataset = dataset
        key = self.name
        metrics = dataset.get(key)
        if metrics is None:
            key = self.name.replace(" ", "")
            metrics = dataset.get(key)
        if metrics is None:
            key = self.unique_id
            metrics = dataset.get(key)

        if metrics is None:
            warnings.warn(f"Metrics for {self.name} not found in dataset")
            self.dataset_key = None
            return

        self.dataset_key = key
        for field_name in ["length_cm", "width_cm", "thickness_cm", "height_cm", "mass_g", "density_kg_m3"]:
            if field_name in metrics:
                target_dict = self.dimensions if field_name.endswith("_cm") else self.material
                if field_name.endswith("_cm"):
                    target_dict[field_name] = metrics[field_name]
                elif field_name == "mass_g":
                    target_dict[field_name] = metrics[field_name]
                elif field_name == "density_kg_m3":
                    target_dict["density"] = metrics[field_name]
                self.metric_sources[field_name] = key

    def validate_metrics(self) -> Dict[str, Tuple[Optional[float], Optional[float]]]:
        if not self.dataset or not self.dataset_key:
            return {}
        dataset_metrics = self.dataset.get(self.dataset_key, {})
        discrepancies = {}
        for field_name in ["length_cm", "width_cm", "thickness_cm", "height_cm", "mass_g", "density_kg_m3"]:
            stored_val = None
            if field_name.endswith("_cm"):
                stored_val = self.dimensions.get(field_name)
            elif field_name == "mass_g":
                stored_val = self.material.get(field_name)
            elif field_name == "density_kg_m3":
                stored_val = self.material.get("density")
            ds_val = dataset_metrics.get(field_name)
            if ds_val is not None and stored_val != ds_val:
                discrepancies[field_name] = (stored_val, ds_val)
        return discrepancies

    def to_fabrication_record(self) -> Dict[str, Any]:
        """Return canonical fabrication record used by validators/exporters."""
        mm_dimensions = self._dimensions_mm()
        physics = dict(self.physics)
        physics.setdefault("mass_kg", self.mass_kg())
        return {
            "name": self.name,
            "latin_name": self.latin_name or self.name,
            "bone_type": self.bone_type,
            "region": self.region,
            "dimensions": mm_dimensions,
            "units": dict(self.units),
            "geometry": dict(self.geometry),
            "material": dict(self.material),
            "physics": physics,
            "connections": dict(self.connections),
            "joint_interfaces": list(self.joint_interfaces),
            "mount_points": list(self.mount_points),
            "manufacturing_notes": list(self.manufacturing_notes),
            "tolerance": dict(self.tolerance),
            "references": list(self.references),
            "revision": self.revision,
            "source_ids": list(self.source_ids),
            "unique_id": self.unique_id,
        }

    def export(self) -> Dict[str, object]:
        data = self.current_state()
        data["source"] = {k: self.metric_sources.get(k, "dataset_default") for k in self.metric_sources}
        return data
