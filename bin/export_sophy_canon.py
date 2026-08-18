#!/usr/bin/env python3
"""Instantiate, validate, and export SOPHY_CANON(H, version)."""

from __future__ import annotations

import argparse
from pathlib import Path

from skeleton.canon import SOPHY_GEOMETRY_CANON_VERSION, instantiate_sophy_canon
from skeleton.exporters.canon_exporter import CanonicalGeometryExporter
from skeleton.validation.canon_validator import CanonicalGeometryValidator


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--height-mm", type=float, required=True, help="master scale H in millimetres")
    parser.add_argument(
        "--canon-version",
        default=SOPHY_GEOMETRY_CANON_VERSION,
        help="explicit canon revision",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("dist/sophy_canonical_geometry.json"),
        help="renderer-neutral JSON output path",
    )
    parser.add_argument(
        "--validation-report",
        type=Path,
        default=Path("reports/sophy_canon_validation_report.json"),
        help="machine-readable invariant report path",
    )
    args = parser.parse_args()

    geometry = instantiate_sophy_canon(args.height_mm, canon_version=args.canon_version)
    validator = CanonicalGeometryValidator(geometry)
    report = validator.run()
    validator.write_report(report, args.validation_report)
    if not report["summary"]["pass"]:
        print(f"canon validation failed: {report['issues']}")
        return 1

    output = CanonicalGeometryExporter(geometry).export(args.output)
    print(
        f"canon_version={geometry.canon_version} "
        f"height_mm={geometry.height_mm} arm_span_mm={geometry.arm_span_mm}"
    )
    print(f"canonical_geometry: {output}")
    print(f"validation_report: {args.validation_report}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
