from __future__ import annotations

from dataclasses import dataclass


@dataclass
class EmotionAgent:
    """Simple valence/arousal state."""

    valence: float = 0.0
    arousal: float = 0.0
    base_lr: float = 0.001

    def modulated_lr(self) -> float:
        return self.base_lr * (1 + self.arousal) * (1 + self.valence)

    def update(self, reward_pred_error: float, surprise: float) -> None:
        self.valence = max(-1.0, min(1.0, self.valence + reward_pred_error))
        self.arousal = max(0.0, min(1.0, self.arousal + surprise))
