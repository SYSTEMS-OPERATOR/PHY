"""Post-Market Surveillance pipeline with anomaly detection."""
from __future__ import annotations

from dataclasses import dataclass
from typing import List


try:
    from sklearn.ensemble import IsolationForest  # type: ignore
    import numpy as np
except Exception:  # pragma: no cover - optional dependency
    IsolationForest = None


@dataclass
class LogEvent:
    value: float


class PMSPipeline:
    def __init__(self) -> None:
        self.log: List[LogEvent] = []
        self.mean: float | None = None
        self.std: float | None = None
        if IsolationForest:
            self.model = IsolationForest(contamination=0.1)
        else:
            self.model = None

    def ingest(self, value: float) -> None:
        self.log.append(LogEvent(value))

    def train(self) -> None:
        values = [e.value for e in self.log]
        if not values:
            return
        if IsolationForest and len(values) > 5:
            data = np.array([[v] for v in values])
            self.model.fit(data)
        self.mean = sum(values) / len(values)
        if len(values) > 1:
            var = sum((v - self.mean) ** 2 for v in values) / (len(values) - 1)
            self.std = var ** 0.5
        else:
            self.std = 0.0

    def detect(self, value: float) -> bool:
        if not self.log:
            return False
        if IsolationForest and self.model:
            pred = self.model.predict([[value]])
            return pred[0] == -1
        if self.mean is None or self.std is None:
            return False
        if self.std == 0:
            return False
        return abs(value - self.mean) > 3 * self.std
