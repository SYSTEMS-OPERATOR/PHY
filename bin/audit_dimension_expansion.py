#!/usr/bin/env python3
"""Report source-backed additions without treating a shortfall as fabricated data."""
import argparse
import json
from pathlib import Path
import sys
import warnings

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from skeleton.bones import load_field
from skeleton.exporters.exporter_agent import json_bytes, sha256
from skeleton.exporters.publication import publish_files
from skeleton.validation.dimension_evidence import load_evidence, expansion_report, markdown_report
from skeleton.validation.engineering_audit import engineering_audit
from skeleton.visualization.studio_model import build_armature


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, default=ROOT)
    parser.add_argument("--require-target", action="store_true",
                        help="Exit 2 when the defensible report does not reach 72; evidence errors exit 1.")
    args = parser.parse_args(argv)
    try:
        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", message="Metrics for .* not found in dataset", module="skeleton.base")
            field = load_field("female_21_baseline")
        catalog, _, _ = load_evidence(ROOT)
        engineering = engineering_audit(field, build_armature()["parts"])
        report = expansion_report(field, engineering)
        files = {"dimension_expansion.json": json_bytes(report),
                 "dimension_expansion.md": markdown_report(report, catalog).encode("utf-8")}
        manifest = {"assignment": report["assignment"], "target_met": report["target_met"],
                    "fabrication_released": False,
                    "files": {name: {"sha256": sha256(data), "size_bytes": len(data)} for name,data in files.items()}}
        files["dimension_expansion_manifest.json"] = json_bytes(manifest)
        destination = args.output_root / "reports"
        publish_files({destination / name: data for name,data in files.items()},
                      destination / "dimension_expansion_manifest.json")
    except (OSError, ValueError, TypeError, KeyError, AttributeError, OverflowError) as error:
        print(json.dumps({"completed": False, "issue": str(error)}, sort_keys=True))
        return 1
    print(json.dumps({"completed": True, "evidence_integrity_pass": True,
        "declared_complete": report["current_complete"], "target_met": report["target_met"],
        "remaining_to_target": report["remaining_to_target"],
        "improved_partial": len(report["improved_partial_ids"]), "record_validation_pass": report["record_validation_pass"],
        "fabrication_released": False}, sort_keys=True))
    return 2 if args.require_target and not report["target_met"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
