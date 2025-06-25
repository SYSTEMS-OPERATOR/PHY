from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict


@dataclass
class HealingState:
    progress: float = 0.0
    last_update: float = 0.0


@dataclass
class HealingEngine:
    """Very small bone healing model."""

    bones: Dict[str, "BoneSpec"]
    states: Dict[str, HealingState] = field(default_factory=dict)
    time: float = 0.0

    def start_healing(self, uid: str) -> None:
        self.states[uid] = HealingState(progress=0.0, last_update=self.time)

    def update(self, dt: float) -> None:
        self.time += dt
        for uid, state in list(self.states.items()):
            elapsed = self.time - state.last_update
            state.progress = min(1.0, state.progress + elapsed / (6 * 7 * 24 * 3600))
            state.last_update = self.time
            bone = self.bones.get(uid)
            if bone is not None:
                base_E = bone.material.get("E_base", 1.0)
                bone.material["E"] = base_E * (0.2 + 0.8 * state.progress)
            if state.progress >= 1.0:
                del self.states[uid]
