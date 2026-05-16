#!/usr/bin/env python3
"""Import reviewed charged-object visual source images.

Run from repository root:

    python PROJECTS/bible/archive/charged_objects/import_charged_object_visuals.py

By default, this imports only entries marked:

    KEEP_READY_FOR_IMPORT

Use --include-comparative to also import entries marked:

    KEEP_COMPARATIVE

The script discovers manifests matching:

    PROJECTS/bible/archive/charged_objects/visual_source_manifest*.json

and downloads each approved image into its declared local_path.

It intentionally skips KEEP_SOURCE_ONLY and SEEK_BETTER_SOURCE entries because
those still need rights/source hardening before binary archive import.
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

ARCHIVE_DIR = Path("PROJECTS/bible/archive/charged_objects")
DEFAULT_READY_STATUSES = {"KEEP_READY_FOR_IMPORT"}
COMPARATIVE_STATUS = "KEEP_COMPARATIVE"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def find_manifests() -> list[Path]:
    return sorted(
        path for path in ARCHIVE_DIR.glob("visual_source_manifest*.json")
        if not path.name.endswith("_import_report.json")
    )


def report_path_for_manifest(manifest_path: Path) -> Path:
    return manifest_path.with_name(f"{manifest_path.stem}_import_report.json")


def download(url: str, dest: Path, overwrite: bool = False, timeout: int = 60) -> dict:
    if dest.exists() and not overwrite:
        return {
            "status": "skipped_exists",
            "path": str(dest),
            "sha256": sha256_file(dest),
            "bytes": dest.stat().st_size,
        }

    dest.parent.mkdir(parents=True, exist_ok=True)
    request = Request(url, headers={"User-Agent": "PHY-Wooden-Armature-Bible-Charged-Object-Importer/0.1"})
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


def should_import(entry: dict, allowed_statuses: set[str]) -> bool:
    return entry.get("review_status") in allowed_statuses


def image_url_for_entry(entry: dict) -> str | None:
    value = entry.get("image_url")
    if value:
        return value
    return entry.get("image_url_candidate")


def local_path_for_entry(entry: dict) -> str | None:
    value = entry.get("local_path")
    if value:
        return value
    return entry.get("local_path_candidate")


def import_manifest(
    manifest_path: Path,
    allowed_statuses: set[str],
    overwrite: bool,
    sleep_seconds: float,
) -> dict:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    results: list[dict] = []

    for entry in manifest.get("entries", []):
        plate_id = entry.get("plate_id")
        review_status = entry.get("review_status")

        if not should_import(entry, allowed_statuses):
            results.append({
                "plate_id": plate_id,
                "review_status": review_status,
                "status": "skipped_review_status",
            })
            continue

        url = image_url_for_entry(entry)
        local_path = local_path_for_entry(entry)
        if not url or not local_path:
            results.append({
                "plate_id": plate_id,
                "review_status": review_status,
                "status": "missing_url_or_path",
            })
            continue

        try:
            result = download(url, Path(local_path), overwrite=overwrite)
            result["plate_id"] = plate_id
            result["review_status"] = review_status
            result["image_url"] = url
        except (HTTPError, URLError, TimeoutError, OSError) as exc:
            result = {
                "plate_id": plate_id,
                "review_status": review_status,
                "image_url": url,
                "status": "error",
                "error": repr(exc),
            }
        results.append(result)
        time.sleep(sleep_seconds)

    report = {
        "manifest": str(manifest_path),
        "allowed_statuses": sorted(allowed_statuses),
        "count": len(results),
        "results": results,
    }
    report_path_for_manifest(manifest_path).write_text(
        json.dumps(report, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--include-comparative", action="store_true")
    parser.add_argument("--sleep", type=float, default=0.5)
    args = parser.parse_args()

    allowed_statuses = set(DEFAULT_READY_STATUSES)
    if args.include_comparative:
        allowed_statuses.add(COMPARATIVE_STATUS)

    manifests = find_manifests()
    if not manifests:
        print("No charged-object visual manifests found.", file=sys.stderr)
        return 1

    reports = []
    error_count = 0
    for manifest_path in manifests:
        print(f"\n=== Importing charged-object manifest {manifest_path} ===")
        report = import_manifest(
            manifest_path=manifest_path,
            allowed_statuses=allowed_statuses,
            overwrite=args.overwrite,
            sleep_seconds=args.sleep,
        )
        reports.append(report)
        for result in report["results"]:
            if result.get("status") == "error":
                error_count += 1

    combined_report = {
        "manifests": [str(path) for path in manifests],
        "allowed_statuses": sorted(allowed_statuses),
        "manifest_count": len(manifests),
        "error_count": error_count,
    }
    combined_path = ARCHIVE_DIR / "charged_object_visuals_import_summary.json"
    combined_path.write_text(json.dumps(combined_report, indent=2, ensure_ascii=False), encoding="utf-8")

    print(json.dumps(combined_report, indent=2, ensure_ascii=False))
    return 1 if error_count else 0


if __name__ == "__main__":
    sys.exit(main())
