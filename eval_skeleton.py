from __future__ import annotations

import argparse
from pathlib import Path

from skeleton.bones import load_bones
from skeleton.datasets import load_dataset
from skeleton.field import SkeletonField

from skvalidators.validator_agent import ValidatorAgent
from reports.report_agent import ReportAgent


def build_skeleton(dataset: str, material: str) -> SkeletonField:
    bones = load_bones(dataset)
    field = SkeletonField(bones)
    for b in bones:
        b.set_material(material)
        b.set_embodiment("physical", b.material)
    return field


def main() -> None:
    parser = argparse.ArgumentParser(description="Validate skeleton metrics")
    parser.add_argument("--material", default="organic", help="Material key")
    parser.add_argument("--export-dir", default="reports", help="Output directory")
    parser.add_argument("--fail-fast", action="store_true", help="Exit non-zero on failure")
    args = parser.parse_args()

    dataset_name = "female_21_baseline"
    data = load_dataset(dataset_name)
    skeleton = build_skeleton(dataset_name, args.material)

    validator = ValidatorAgent(skeleton, data)
    results = validator.run_all_checks()
    if args.fail_fast and not results["summary"]["pass"]:
        raise SystemExit("Validation failed")

    reporter = ReportAgent(skeleton, results)
    reporter.export(args.export_dir)


if __name__ == "__main__":
    main()
