from __future__ import annotations

from dataclasses import dataclass, field
from typing import Tuple

import numpy as np


@dataclass
class TactileArray:
    """Simple tactile grid."""

    cells: int = 16
    noise_std: float = 0.1
    pressures: np.ndarray = field(init=False)

    def __post_init__(self) -> None:
        self.pressures = np.zeros(self.cells)

    def read(self) -> np.ndarray:
        noisy = self.pressures + np.random.normal(scale=self.noise_std, size=self.cells)
        return np.clip(noisy, 0.0, None)
