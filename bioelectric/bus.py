from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Iterable


@dataclass
class BioElectricBus:
    """Minimal EMG/EKG distribution network."""

    nodes: Iterable[str]
    voltages: Dict[str, float] = field(init=False)

    def __post_init__(self) -> None:
        self.voltages = {n: 0.0 for n in self.nodes}

    def distribute(self, signals: Dict[str, float]) -> None:
        for n, v in signals.items():
            if n in self.voltages:
                self.voltages[n] = v

    def inject_ekg(self, waveform: Iterable[float]) -> None:
        # placeholder for sync pulses
        pass
