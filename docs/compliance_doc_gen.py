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
        content = "%PDF-1.4\n" + json.dumps(data, indent=2)
        self.out_pdf.write_text(content)

