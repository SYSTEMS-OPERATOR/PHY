from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Iterable

import numpy as np
from scipy.signal import butter, lfilter


@dataclass
class ControlAgent:
    """Process EMG samples and output muscle activations."""

    muscle_names: Iterable[str]
    fc_hz: float = 6.0
    _z: np.ndarray = field(init=False)
    _b: np.ndarray = field(init=False)
    _a: np.ndarray = field(init=False)
    activations: Dict[str, float] = field(init=False)

    def __post_init__(self) -> None:
        self.activations = {n: 0.0 for n in self.muscle_names}
        self._b, self._a = butter(4, self.fc_hz, fs=1000, btype="low")
        self._z = np.zeros(max(len(self._a), len(self._b)) - 1)

    def update(self, dt: float, emg_map: Dict[str, Iterable[float]]) -> Dict[str, float]:
        for name, samples in emg_map.items():
            data = np.abs(np.array(list(samples), dtype=float))
            if data.size == 0:
                continue
            filt, self._z = lfilter(self._b, self._a, data, zi=self._z)
            self.activations[name] = float(np.clip(filt[-1], 0.0, 1.0))
        return dict(self.activations)
