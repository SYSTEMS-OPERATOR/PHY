"""Torque tracking validation."""

import json
from pathlib import Path


def trace(forces, out: Path) -> None:
    avg = sum(forces) / max(len(forces), 1)
    data = {"mean": avg}
    out.write_text(json.dumps(data))
