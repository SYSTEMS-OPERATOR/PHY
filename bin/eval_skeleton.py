from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from skeleton.bones import load_bones
from skeleton.datasets import load_dataset
from validators.validator_agent import ValidatorAgent
from reports.report_agent import ReportAgent


def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluate skeleton integrity")
    parser.add_argument("--material", default="organic", help="material key")
    parser.add_argument("--export-dir", default="reports", help="output dir")
    parser.add_argument("--fail-fast", action="store_true", help="exit on fail")
    args = parser.parse_args()

    dataset = load_dataset("female_21_baseline")
    bones = load_bones("female_21_baseline")
    for b in bones:
        b.set_material(args.material)
    va = ValidatorAgent(bones, dataset)
    va.run_all_checks()
    ReportAgent(bones, va.results).export(args.export_dir)
    if args.fail_fast and not va.results["summary"]["pass"]:
        raise SystemExit("Validation failed")


if __name__ == "__main__":
    main()

