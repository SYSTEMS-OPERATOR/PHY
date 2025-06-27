"""Post-Market Surveillance pipeline with anomaly detection."""
from __future__ import annotations

from dataclasses import dataclass
from typing import List

from sklearn.ensemble import IsolationForest
import numpy as np


@dataclass
class LogEvent:
    value: float


class PMSPipeline:
    def __init__(self) -> None:
        self.model = IsolationForest(contamination=0.1)
        self.log: List[LogEvent] = []

    def ingest(self, value: float) -> None:
        self.log.append(LogEvent(value))

    def train(self) -> None:
        data = np.array([[e.value] for e in self.log])
        if len(data) > 5:
            self.model.fit(data)

    def detect(self, value: float) -> bool:
        if not self.log:
            return False
        pred = self.model.predict([[value]])
        return pred[0] == -1
