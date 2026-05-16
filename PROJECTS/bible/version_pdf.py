#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
from datetime import datetime, timezone
from pathlib import Path

PDF = Path("PROJECTS/reports/WOODEN_ARMATURE_BIBLE.pdf")
PDF_REPORT = Path("PROJECTS/reports/WOODEN_ARMATURE_BIBLE.pdf_report.json")
VERSIONS = Path("PROJECTS/reports/pdf_versions")
MANIFEST = VERSIONS / "manifest.json"


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def load_manifest() -> dict:
    if MANIFEST.exists():
        return json.loads(MANIFEST.read_text(encoding="utf-8"))
    return {"versions": []}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--version", required=True, help="Example: v001_original_relic_draft")
    parser.add_argument("--note", default="", help="Short note describing this version")
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()

    if not PDF.exists():
        raise SystemExit(f"Missing PDF: {PDF}")

    VERSIONS.mkdir(parents=True, exist_ok=True)
    base = f"WOODEN_ARMATURE_BIBLE_{args.version}"
    out_pdf = VERSIONS / f"{base}.pdf"
    out_report = VERSIONS / f"{base}.pdf_report.json"
    out_meta = VERSIONS / f"{base}.version.json"

    if out_pdf.exists() and not args.overwrite:
        raise SystemExit(f"Version already exists: {out_pdf}. Use --overwrite to replace.")

    shutil.copy2(PDF, out_pdf)
    if PDF_REPORT.exists():
        shutil.copy2(PDF_REPORT, out_report)

    pdf_report = None
    if PDF_REPORT.exists():
        pdf_report = json.loads(PDF_REPORT.read_text(encoding="utf-8"))

    meta = {
        "version": args.version,
        "note": args.note,
        "source_pdf": str(PDF),
        "version_pdf": str(out_pdf),
        "source_report": str(PDF_REPORT) if PDF_REPORT.exists() else None,
        "version_report": str(out_report) if PDF_REPORT.exists() else None,
        "sha256": sha256(out_pdf),
        "bytes": out_pdf.stat().st_size,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "source_pdf_report": pdf_report,
    }
    out_meta.write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")

    manifest = load_manifest()
    manifest["versions"] = [v for v in manifest.get("versions", []) if v.get("version") != args.version]
    manifest["versions"].append(meta)
    manifest["versions"].sort(key=lambda x: x.get("version", ""))
    MANIFEST.write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")

    print(json.dumps(meta, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
