#!/usr/bin/env python3
"""Inspect spine dimensions/features offline; does not qualify fabrication."""
import argparse
import json
from pathlib import Path
import sys
import warnings

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from skeleton.bones import load_field
from skeleton.validation.spine_audit import audit_spine, write_reports


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", default="female_21_baseline")
    parser.add_argument("--output-root", type=Path, default=ROOT)
    parser.add_argument("--require-consistent", action="store_true", help="Exit 2 for data inconsistencies; physical unknowns remain explicit.")
    args = parser.parse_args(argv)
    try:
        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", message="Metrics for .* not found in dataset", module="skeleton.base")
            report = audit_spine(load_field(args.dataset))
        paths = write_reports(report, args.output_root)
    except (OSError, ValueError, TypeError, AttributeError, KeyError, OverflowError, RecursionError) as error:
        print(json.dumps({"completed": False, "issue": str(error)}, sort_keys=True))
        return 1
    print(json.dumps({"completed": True, "counts": report["counts"], "consistency_pass": report["consistency_pass"],
                      "all_dimensions_verified": False, "fabrication_released": False, "paths": paths}, sort_keys=True))
    return 2 if args.require_consistent and not report["consistency_pass"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
