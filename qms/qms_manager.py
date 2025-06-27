"""Quality Management System manager for controlled documents and CAPA."""
from __future__ import annotations

import datetime
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List


@dataclass
class ControlledDocument:
    path: Path
    version: str
    signer: str
    timestamp: datetime.datetime


@dataclass
class CAPA:
    issue_id: str
    severity: str
    owner: str
    due: datetime.datetime
    opened: datetime.datetime = field(
        default_factory=lambda: datetime.datetime.now(datetime.timezone.utc)
    )
    closed: datetime.datetime | None = None

    def close(self) -> None:
        self.closed = datetime.datetime.now(datetime.timezone.utc)


class QMSManager:
    """Simplified QMS manager tracking documents and CAPA events."""

    def __init__(self, repo_root: Path) -> None:
        self.repo_root = repo_root
        self.docs: Dict[str, ControlledDocument] = {}
        self.capa_events: Dict[str, CAPA] = {}

    def audit_trail_pdf(self, out_path: Path) -> None:
        """Export audit trail to a PDF file."""
        try:
            from reportlab.lib.pagesizes import letter
            from reportlab.pdfgen import canvas
        except Exception:  # pragma: no cover - optional dependency
            return

        c = canvas.Canvas(str(out_path), pagesize=letter)
        text = c.beginText(40, 750)
        text.textLine("Audit Trail")
        for doc in self.docs.values():
            text.textLine(f"DOC {doc.path} {doc.version} {doc.signer}")
        for capa in self.capa_events.values():
            status = "closed" if capa.closed else "open"
            text.textLine(
                f"CAPA {capa.issue_id} {status} {capa.severity} due {capa.due.date()}"
            )
        c.drawText(text)
        c.showPage()
        c.save()

    def add_document(self, doc_path: str, signer: str) -> None:
        path = self.repo_root / doc_path
        version = datetime.datetime.now(datetime.timezone.utc).isoformat()
        self.docs[doc_path] = ControlledDocument(path, version, signer, datetime.datetime.now(datetime.timezone.utc))

    def raise_capa(
        self, issue_id: str, severity: str, owner: str, days: int = 30
    ) -> CAPA:
        """Create a CAPA entry with a due date."""
        due = datetime.datetime.now(datetime.timezone.utc) + datetime.timedelta(
            days=days
        )
        capa = CAPA(issue_id=issue_id, severity=severity, owner=owner, due=due)
        self.capa_events[issue_id] = capa
        return capa

    def close_capa(self, issue_id: str) -> None:
        if issue_id in self.capa_events:
            self.capa_events[issue_id].close()

    def export_dhf(self) -> List[str]:
        """Export a simple Design History File log."""
        lines = []
        for doc in self.docs.values():
            lines.append(f"{doc.path}:{doc.version}:{doc.signer}")
        for capa in self.capa_events.values():
            status = "closed" if capa.closed else "open"
            lines.append(
                f"CAPA {capa.issue_id}:{status}:{capa.severity}:due={capa.due.isoformat()}"
            )
        return lines
