from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Tuple

import numpy as np


@dataclass
class AuditoryAgent:
    """Simulated cochlear filterbank."""

    n_bands: int = 32

    def process(self, waveform: Iterable[float]) -> Tuple[np.ndarray, np.ndarray]:
        """Return magnitude and ITD/ILD cues (dummy)."""
        data = np.array(list(waveform), dtype=float)
        if data.size == 0:
            return np.zeros(self.n_bands), np.zeros(self.n_bands)
        spec = np.abs(np.fft.rfft(data))
        bands = np.interp(
            np.linspace(0, spec.size - 1, self.n_bands),
            np.arange(spec.size),
            spec,
        )
        itd_ild = np.zeros_like(bands)
        return bands, itd_ild
