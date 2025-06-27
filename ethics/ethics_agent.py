"""Ethics agent performing risk-benefit analysis and event reporting."""
from __future__ import annotations

import datetime
from dataclasses import dataclass
from typing import List

try:
    from reportlab.lib.pagesizes import letter
    from reportlab.pdfgen import canvas
except Exception:  # pragma: no cover - optional dependency
    canvas = None


@dataclass
class AdverseEvent:
    timestamp: datetime.datetime
    description: str


class EthicsAgent:
    def __init__(self, threshold: float = 0.3) -> None:
        self.threshold = threshold
        self.events: List[AdverseEvent] = []

    def assess_risk(self, risk: float, benefit: float) -> bool:
        ratio = risk / benefit if benefit else 1.0
        return ratio >= self.threshold

    def pre_trial_check(self, risk: float, benefit: float) -> bool:
        """Return True if trial should proceed."""
        if self.assess_risk(risk, benefit):
            return True
        self.record_event("Risk exceeds benefit; trial stopped")
        # send notification (mock)
        print("IRB ALERT")
        return False

    def record_event(self, description: str) -> None:
        self.events.append(AdverseEvent(datetime.datetime.utcnow(), description))

    def export_medwatch_pdf(self, out: str) -> None:
        if not canvas:
            return
        c = canvas.Canvas(out, pagesize=letter)
        text = c.beginText(40, 750)
        text.textLine("MedWatch 3500A")
        for e in self.events:
            text.textLine(f"{e.timestamp.isoformat()} {e.description}")
        c.drawText(text)
        c.showPage()
        c.save()

    def generate_medwatch_xml(self) -> str:
        return "<medwatch><events>{}</events></medwatch>".format(len(self.events))
