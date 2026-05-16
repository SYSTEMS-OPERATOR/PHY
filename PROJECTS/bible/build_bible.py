#!/usr/bin/env python3
"""Compile the Wooden Armature Bible Markdown artifact.

Run from repository root:

    python PROJECTS/bible/build_bible.py

The script reads PROJECTS/bible/build_order.json, concatenates the listed
Markdown sources, and writes PROJECTS/reports/WOODEN_ARMATURE_BIBLE.md.

It intentionally does not embed images yet. Image embedding and PDF layout are
future passes after plate rights and layout review.
"""

from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path

BUILD_ORDER_PATH = Path("PROJECTS/bible/build_order.json")


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8").strip() + "\n"


def compile_bible(config: dict) -> tuple[str, list[str]]:
    source_root = Path(config["source_root"])
    missing: list[str] = []
    chunks: list[str] = []

    generated_at = datetime.now(timezone.utc).isoformat()
    chunks.append(
        "<!--\n"
        f"Generated artifact: {config.get('artifact_name', 'WOODEN_ARMATURE_BIBLE')}\n"
        f"Generated at UTC: {generated_at}\n"
        "Source: PROJECTS/bible/build_order.json\n"
        "Note: This is a compiled Markdown draft. Image embedding and PDF layout are future passes.\n"
        "-->\n"
    )

    for entry in config["sections"]:
        relative = entry["path"]
        title = entry.get("title", relative)
        path = source_root / relative
        if not path.exists():
            missing.append(str(path))
            chunks.append(f"\n\n<!-- MISSING SOURCE: {path} ({title}) -->\n")
            continue

        chunks.append("\n\n---\n\n")
        chunks.append(f"<!-- SOURCE: {path} -->\n\n")
        chunks.append(read_text(path))

    return "".join(chunks).rstrip() + "\n", missing


def main() -> int:
    config = read_json(BUILD_ORDER_PATH)
    output_path = Path(config["output_path"])
    output_path.parent.mkdir(parents=True, exist_ok=True)

    content, missing = compile_bible(config)
    output_path.write_text(content, encoding="utf-8")

    report_path = output_path.with_suffix(".build_report.json")
    report = {
        "artifact_name": config.get("artifact_name"),
        "output_path": str(output_path),
        "section_count": len(config.get("sections", [])),
        "missing_sources": missing,
    }
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")

    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 1 if missing else 0


if __name__ == "__main__":
    raise SystemExit(main())
