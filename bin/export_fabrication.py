#!/usr/bin/env python3
from __future__ import annotations

from skeleton.bones import load_field
from skeleton.exporters.exporter_agent import ExporterAgent


def main() -> int:
    field = load_field("female_21_baseline")
    paths = ExporterAgent(field).export_all()
    for name, path in paths.items():
        print(f"{name}: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
