#!/usr/bin/env python3
"""Check the explicit NIH crosswalk against a pinned PHY-REF audit package."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from skeleton.anatomical_inventory import expected_bones
from skeleton.exporters.exporter_agent import json_bytes, sha256
from skeleton.exporters.publication import publish_files
from skeleton.validation.dimension_evidence import read_json, _pinned_file
from skeleton.validation.reference_correspondence import audit_correspondence


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--reference-root", type=Path, required=True)
    p.add_argument("--output-root", type=Path, default=ROOT)
    args = p.parse_args(argv)
    try:
        catalog = read_json(ROOT / "references/nih_reference_correspondence.json")
        path = _pinned_file(args.reference_root, {
            "path": catalog["audit_path"], "sha256": catalog["audit_sha256"]})
        report = audit_correspondence(catalog, read_json(path),
            hashlib.sha256(path.read_bytes()).hexdigest(),
            {row["bone_id"] for row in expected_bones()})
        destination = args.output_root / "reports"
        payload = json_bytes(report)
        manifest = json_bytes({"fabrication_released": False,
            "files": {"nih_correspondence_audit.json": {
                "sha256": sha256(payload), "size_bytes": len(payload)}}})
        publish_files({destination / "nih_correspondence_audit.json": payload,
                       destination / "nih_correspondence_manifest.json": manifest},
                      destination / "nih_correspondence_manifest.json")
    except (ValueError, TypeError, KeyError, OSError) as error:
        print(json.dumps({"completed": False, "issue": str(error)}))
        return 1
    print(json.dumps({"completed": True, "matched_ids": report["matched_canonical_ids"],
                      "fabrication_released": False}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
