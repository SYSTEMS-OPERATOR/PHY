from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Any
from reportlab.lib.pagesizes import letter
from reportlab.pdfgen import canvas


@dataclass
class ComplianceDocGen:
    """Generate compliance documents from YAML/JSON logs."""

    risk_yaml: Path
    test_log: Path
    out_pdf: Path

    def generate(self) -> None:
        data: Dict[str, Any] = {
            "risk": self.risk_yaml.read_text(),
            "tests": json.loads(self.test_log.read_text()),
        }
        c = canvas.Canvas(str(self.out_pdf), pagesize=letter)
        text = c.beginText(50, 750)
        for key, val in data.items():
            text.textLine(f"{key}: {val}")
        c.drawText(text)
        c.showPage()
        c.save()
