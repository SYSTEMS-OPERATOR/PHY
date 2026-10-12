#!/usr/bin/env python3
"""Review structural craniofacial relationships and feature-boundary unknowns."""
import argparse
import json
from pathlib import Path
import sys
import warnings

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from skeleton.bones import load_field
from skeleton.validation.craniofacial_audit import craniofacial_audit, write_reports
from skeleton.visualization.studio_model import build_armature


def main(argv=None):
    parser = argparse.ArgumentParser(description="Offline craniofacial review; no mechanical or fabrication qualification.")
    parser.add_argument("--dataset", default="female_21_baseline")
    parser.add_argument("--height-mm", type=float, default=1676.4,
                        help="Reference-body display stature; does not modify SOPHY canon.")
    parser.add_argument("--output-root", type=Path, default=Path("."))
    args = parser.parse_args(argv)
    try:
        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", message="Metrics for .* not found in dataset", module="skeleton.base")
            field = load_field(args.dataset)
        model = build_armature(height_mm=args.height_mm, span_equals_height=True)
        report = craniofacial_audit(field, model)
        paths = write_reports(report, args.output_root)
    except (OSError, ValueError, TypeError, AttributeError, KeyError, OverflowError, RecursionError) as error:
        print(json.dumps({"completed": False, "issue": str(error)}, sort_keys=True, allow_nan=False))
        return 1
    print(json.dumps({"completed": True, "fabrication_released": False,
                      "counts": report["counts"], "paths": paths}, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
