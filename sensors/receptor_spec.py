from __future__ import annotations

from dataclasses import dataclass
from typing import Dict


@dataclass
class ReceptorSpec:
    """Configuration for a single sensory receptor."""

    name: str
    type: str  # 'muscle_spindle', 'GTO', 'nociceptor', 'joint_capsule'
    threshold: float
    location: Dict[str, str]
    signal_gain: float = 1.0
