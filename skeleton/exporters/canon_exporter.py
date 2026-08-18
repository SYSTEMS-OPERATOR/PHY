"""Renderer- and fabrication-neutral SOPHY canon export."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from skeleton.canon.geometry import CanonicalGeometry
from skeleton.validation.canon_validator import CanonicalGeometryValidator


@dataclass(frozen=True, slots=True)
class CanonicalGeometryExporter:
    geometry: CanonicalGeometry

    def export(
        self,
        path: Path = Path("dist/sophy_canonical_geometry.json"),
    ) -> Path:
        report = CanonicalGeometryValidator(self.geometry).run()
        if not report["summary"]["pass"]:
            raise ValueError(f"canonical geometry failed validation: {report['issues']}")
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(self.geometry.to_json(), encoding="utf-8")
        return path
