from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Tuple
import numpy as np


@dataclass
class MemoryAgent:
    """Simple circular buffer of experiences."""

    capacity: int = 10000
    buffer: List[Tuple[np.ndarray, np.ndarray, float, np.ndarray]] = field(init=False)
    idx: int = field(default=0, init=False)

    def __post_init__(self) -> None:
        self.buffer = []

    def add(self, obs: np.ndarray, act: np.ndarray, reward: float, next_obs: np.ndarray) -> None:
        if len(self.buffer) < self.capacity:
            self.buffer.append((obs, act, reward, next_obs))
        else:
            self.buffer[self.idx] = (obs, act, reward, next_obs)
            self.idx = (self.idx + 1) % self.capacity
