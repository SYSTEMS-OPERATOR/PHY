#!/usr/bin/env python3
"""Import all anatomical plate images from every plate manifest.

Run from repository root:

    python PROJECTS/bible/archive/anatomical_plates/import_all_anatomical_plates.py

This discovers all files matching:

    PROJECTS/bible/archive/anatomical_plates/plate_manifest*.json

and invokes import_anatomical_plates.py for each one.

Use --overwrite to refresh existing local images.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

ARCHIVE_DIR = Path("PROJECTS/bible/archive/anatomical_plates")
IMPORTER = ARCHIVE_DIR / "import_anatomical_plates.py"


def find_manifests() -> list[Path]:
    return sorted(
        path for path in ARCHIVE_DIR.glob("plate_manifest*.json")
        if not path.name.endswith("_import_report.json")
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--sleep", type=float, default=0.5)
    args = parser.parse_args()

    manifests = find_manifests()
    if not manifests:
        print("No plate manifests found.", file=sys.stderr)
        return 1

    failures: list[Path] = []
    for manifest in manifests:
        cmd = [
            sys.executable,
            str(IMPORTER),
            "--manifest",
            str(manifest),
            "--sleep",
            str(args.sleep),
        ]
        if args.overwrite:
            cmd.append("--overwrite")

        print("\n=== Importing", manifest, "===")
        completed = subprocess.run(cmd, check=False)
        if completed.returncode != 0:
            failures.append(manifest)

    if failures:
        print("\nFailed manifests:", file=sys.stderr)
        for failure in failures:
            print(f"- {failure}", file=sys.stderr)
        return 1

    print("\nAll manifests imported successfully.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
