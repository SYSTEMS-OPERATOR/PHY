from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict


@dataclass
class BioElectricBus:
    voltages: Dict[str, float] = field(default_factory=dict)

    def inject_ekg(self, waveform: float) -> None:
        for node in self.voltages:
            self.voltages[node] = waveform
