#!/usr/bin/env python3
"""Inspect engineering coverage offline, independently of fabrication export."""
import argparse
import json
from pathlib import Path
import sys
import warnings

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from skeleton.bones import load_field
from skeleton.validation.engineering_audit import engineering_audit, write_reports
from skeleton.visualization.studio_model import build_armature


def main(argv=None):
    parser = argparse.ArgumentParser(description="Audit adult engineering evidence; does not qualify fabrication.")
    parser.add_argument("--dataset", default="female_21_baseline")
    parser.add_argument("--output-root", type=Path, default=Path("."))
    args = parser.parse_args(argv)
    try:
        with warnings.catch_warnings():
            # Every missing binding is retained explicitly in the audit.
            warnings.filterwarnings("ignore", message="Metrics for .* not found in dataset", module="skeleton.base")
            field = load_field(args.dataset)
        report = engineering_audit(field, build_armature()["parts"])
        paths = write_reports(report, args.output_root)
    except (OSError, ValueError, TypeError, AttributeError, KeyError, OverflowError, RecursionError) as error:
        print(json.dumps({"completed": False, "issue": str(error)}, sort_keys=True, allow_nan=False))
        return 1
    print(json.dumps({"completed": True, "fabrication_released": False,
                      "metrics": {key: value["count"] for key, value in report["metrics"].items()},
                      "paths": paths}, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
