"""Post-Market Surveillance pipeline with anomaly detection."""
from __future__ import annotations

from dataclasses import dataclass
from typing import List

import json
try:
    import requests
except Exception:  # pragma: no cover - optional dependency
    class _DummyRequests:
        def get(self, *args, **kwargs):
            raise RuntimeError("requests not available")

    requests = _DummyRequests()


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
        self.cve_seen: set[str] = set()

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

    def watch_cves(self, deps: List[str]) -> List[str]:
        """Check NVD feed for vulnerabilities impacting deps."""
        try:
            r = requests.get(
                "https://services.nvd.nist.gov/rest/json/cves/2.0?resultsPerPage=5",
                timeout=5,
            )
            data = r.json()
        except Exception:
            return []

        hits = []
        for item in data.get("vulnerabilities", []):
            cve = item.get("cve", {}).get("id")
            if not cve or cve in self.cve_seen:
                continue
            self.cve_seen.add(cve)
            desc = json.dumps(item)
            if any(dep.lower() in desc.lower() for dep in deps):
                hits.append(cve)
        return hits
