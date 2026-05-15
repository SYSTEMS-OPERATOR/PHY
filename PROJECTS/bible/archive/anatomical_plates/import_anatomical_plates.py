#!/usr/bin/env python3
"""Import anatomical plate images listed in a manifest JSON file.

Run from repository root:

    python PROJECTS/bible/archive/anatomical_plates/import_anatomical_plates.py

Import the systems batch:

    python PROJECTS/bible/archive/anatomical_plates/import_anatomical_plates.py \
      --manifest PROJECTS/bible/archive/anatomical_plates/plate_manifest_batch2_systems.json

The script downloads source images into their declared local paths and writes a
small import report. It is intentionally conservative: it will not overwrite an
existing image unless --overwrite is passed.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

DEFAULT_MANIFEST_PATH = Path("PROJECTS/bible/archive/anatomical_plates/plate_manifest.json")
DEFAULT_REPORT_PATH = Path("PROJECTS/bible/archive/anatomical_plates/import_report.json")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def report_path_for_manifest(manifest_path: Path) -> Path:
    stem = manifest_path.stem
    return manifest_path.with_name(f"{stem}_import_report.json")


def download(url: str, dest: Path, overwrite: bool = False, timeout: int = 60) -> dict:
    if dest.exists() and not overwrite:
        return {
            "status": "skipped_exists",
            "path": str(dest),
            "sha256": sha256_file(dest),
            "bytes": dest.stat().st_size,
        }

    dest.parent.mkdir(parents=True, exist_ok=True)
    request = Request(url, headers={"User-Agent": "PHY-Wooden-Armature-Bible-Importer/0.2"})
    with urlopen(request, timeout=timeout) as response:
        data = response.read()
        content_type = response.headers.get("Content-Type")

    dest.write_bytes(data)
    return {
        "status": "downloaded",
        "path": str(dest),
        "sha256": hashlib.sha256(data).hexdigest(),
        "bytes": len(data),
        "content_type": content_type,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", default=str(DEFAULT_MANIFEST_PATH))
    parser.add_argument("--report", default=None)
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--sleep", type=float, default=0.5, help="delay between downloads")
    args = parser.parse_args()

    manifest_path = Path(args.manifest)
    report_path = Path(args.report) if args.report else report_path_for_manifest(manifest_path)

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    results = []

    for plate in manifest.get("plates", []):
        url = plate.get("image_url")
        local_path = plate.get("local_path")
        if not url or not local_path:
            results.append({"plate_id": plate.get("plate_id"), "status": "missing_url_or_path"})
            continue

        try:
            result = download(url, Path(local_path), overwrite=args.overwrite)
            result["plate_id"] = plate.get("plate_id")
            result["image_url"] = url
        except (HTTPError, URLError, TimeoutError, OSError) as exc:
            result = {
                "plate_id": plate.get("plate_id"),
                "image_url": url,
                "status": "error",
                "error": repr(exc),
            }
        results.append(result)
        time.sleep(args.sleep)

    report = {
        "manifest": str(manifest_path),
        "count": len(results),
        "results": results,
    }
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
