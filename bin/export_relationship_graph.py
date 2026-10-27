#!/usr/bin/env python3
"""Export a fresh offline relationship snapshot; no anatomy or routing adoption."""
import argparse
import json
from pathlib import Path
import sys
import warnings

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from skeleton.bones import load_field
from skeleton.relationships import build_relationship_snapshot
from skeleton.exporters.relationship_exporter import export_relationship_snapshot


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, default=Path("dist/relationships"))
    parser.add_argument("--dataset", default="female_21_baseline")
    args = parser.parse_args(argv)
    try:
        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", message="Metrics for .* not found in dataset", module="skeleton.base")
            graph = build_relationship_snapshot(load_field(args.dataset))
        paths = export_relationship_snapshot(graph, args.output_root)
    except (OSError, ValueError, TypeError, AttributeError, KeyError, OverflowError, RecursionError) as error:
        print(json.dumps({"published": False, "error": str(error)}, sort_keys=True))
        return 1
    print(json.dumps({"published": True, "paths": paths, "inventory": graph["inventory"],
                      "fabrication_released": False}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
