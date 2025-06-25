from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict

@dataclass
class MuscleSpec:
    """Minimal muscle description linking two bone mount points."""

    name: str
    origin: Dict[str, str]
    insertion: Dict[str, str]
    max_isometric_force_N: float
    optimal_fiber_len_cm: float
    tendon_slack_len_cm: float
    pennation_angle_deg: float = 0.0
    activation: float = 0.0
    state: Dict[str, float] = field(default_factory=dict)
    reference: str = "AnyBody/Hill-type defaults"
