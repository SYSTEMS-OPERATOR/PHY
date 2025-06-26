from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np


@dataclass
class CerebellumForwardModel:
    """Predict next state from current state and motor command."""

    state_dim: int
    action_dim: int
    lr: float = 0.01
    W: np.ndarray = field(init=False)

    def __post_init__(self) -> None:
        self.W = np.zeros((self.state_dim + self.action_dim, self.state_dim))

    def predict(self, state: np.ndarray, action: np.ndarray) -> np.ndarray:
        x = np.concatenate([state, action])
        return x @ self.W

    def update(self, state: np.ndarray, action: np.ndarray, next_state: np.ndarray) -> float:
        pred = self.predict(state, action)
        err = next_state - pred
        x = np.concatenate([state, action])[:, None]
        self.W += self.lr * x @ err[None, :]
        return float((err ** 2).mean())
