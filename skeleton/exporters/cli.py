"""Shared export/assembly entrypoint; failed attempts never masquerade as packages."""
import argparse
import json
from pathlib import Path
import warnings

from skeleton.bones import load_field
from .exporter_agent import ExportError, ExporterAgent, json_bytes
from .publication import publish_files


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Export a canonical record package with explicit validation status.")
    parser.add_argument("--mode", choices=("review", "validated"), default="validated",
                        help="validated (default) requires record and identity gates; review permits known gaps")
    parser.add_argument("--dataset", default="female_21_baseline")
    parser.add_argument("--output-root", type=Path, default=Path("."))
    args = parser.parse_args(argv)
    root = args.output_root
    try:
        with warnings.catch_warnings():
            # These missing bindings are enumerated in the package/attempt report.
            warnings.filterwarnings("ignore", message="Metrics for .* not found in dataset", module="skeleton.base")
            field = load_field(args.dataset)
        paths = ExporterAgent(field).export_all(root / "dist", root / "reports", root / "exports", mode=args.mode)
    except (OSError, ValueError, TypeError, AttributeError, KeyError, OverflowError, RecursionError) as error:
        report = error.report if isinstance(error, ExportError) else {
            "mode": args.mode, "published": False, "code": "input_failed", "issues": [{"issue": str(error)}],
        }
        failure = root / "reports" / "export_failure.json"
        response = {"mode": args.mode, "published": False, "code": report["code"], "diagnostic_report": str(failure)}
        try:
            publish_files({failure: json_bytes(report)}, failure)
        except (OSError, ValueError, TypeError, RecursionError) as diagnostic_error:
            response.update(diagnostic_report=None, diagnostic_error=str(diagnostic_error))
        print(json.dumps(response, allow_nan=False, sort_keys=True))
        return 1
    manifest = json.loads(Path(paths["manifest"]).read_text(encoding="utf-8"))
    print(json.dumps({"mode": args.mode, "published": True,
                      "validated_ready": manifest["validated_ready"], "fabrication_released": False,
                      "paths": paths}, allow_nan=False, sort_keys=True))
    return 0
