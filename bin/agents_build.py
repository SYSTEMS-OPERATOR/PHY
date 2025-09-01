#!/usr/bin/env python3
from __future__ import annotations
import json, pathlib, sys
import yaml

ROOT = pathlib.Path(__file__).resolve().parents[1]
REG = ROOT / "meta" / "agents.yml"
OUT = ROOT / "dist" / "agents.index.json"
OUT.parent.mkdir(parents=True, exist_ok=True)

if not REG.exists():
    print("missing meta/agents.yml", file=sys.stderr)
    sys.exit(1)

agents = yaml.safe_load(REG.read_text())
OUT.write_text(json.dumps({"agents": agents}, indent=2))
print("wrote", OUT)
