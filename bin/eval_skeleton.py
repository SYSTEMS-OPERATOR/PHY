#!/usr/bin/env python
from __future__ import annotations

import argparse
from pathlib import Path

from skeleton.bones import load_bones
from skeleton.datasets import load_dataset
from skeleton.field import SkeletonField
from validators.validator_agent import ValidatorAgent
from reports.report_agent import ReportAgent


def main() -> None:
    parser = argparse.ArgumentParser(description="Validate skeleton")
    parser.add_argument("--material", default="organic")
    parser.add_argument("--export-dir", default="reports")
    parser.add_argument("--fail-fast", action="store_true")
    args = parser.parse_args()

    dataset = load_dataset("female_21_baseline")
    bones = load_bones("female_21_baseline")
    for b in bones:
        b.set_material(args.material)
        b.set_embodiment("physical")
    field = SkeletonField(bones)

    validator = ValidatorAgent(field, dataset)
    validator.run_all_checks()
    ReportAgent(field, validator.results).export(args.export_dir)

    if args.fail_fast and not validator.results["summary"]["pass"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
