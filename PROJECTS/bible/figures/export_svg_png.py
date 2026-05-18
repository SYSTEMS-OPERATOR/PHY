#!/usr/bin/env python3
from __future__ import annotations

import json
import subprocess
from datetime import datetime, timezone
from pathlib import Path

FIGURES_DIR = Path("PROJECTS/bible/figures")
REPORT = FIGURES_DIR / "png_export_report.json"


def candidate_svgs() -> list[Path]:
    return sorted(
        p for p in FIGURES_DIR.glob("*.svg")
        if p.is_file()
    )


def export_with_cairosvg(svg_path: Path, png_path: Path) -> dict:
    import cairosvg
    cairosvg.svg2png(
        url=str(svg_path),
        write_to=str(png_path),
        output_width=1800,
        output_height=None,
    )
    return {
        "status": "exported",
        "method": "cairosvg",
        "svg": str(svg_path),
        "png": str(png_path),
        "bytes": png_path.stat().st_size,
    }


def export_with_rsvg(svg_path: Path, png_path: Path) -> dict:
    subprocess.run(
        ["rsvg-convert", "-w", "1800", "-o", str(png_path), str(svg_path)],
        check=True,
        capture_output=True,
        text=True,
    )
    return {
        "status": "exported",
        "method": "rsvg-convert",
        "svg": str(svg_path),
        "png": str(png_path),
        "bytes": png_path.stat().st_size,
    }


def export_one(svg_path: Path) -> dict:
    png_path = svg_path.with_suffix(".png")
    try:
        return export_with_cairosvg(svg_path, png_path)
    except Exception as first_error:
        try:
            return export_with_rsvg(svg_path, png_path)
        except Exception as second_error:
            return {
                "status": "error",
                "svg": str(svg_path),
                "png": str(png_path),
                "cairosvg_error": repr(first_error),
                "rsvg_error": repr(second_error),
            }


def main() -> int:
    results = [export_one(path) for path in candidate_svgs()]
    report = {
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "source_dir": str(FIGURES_DIR),
        "count": len(results),
        "exported": sum(1 for item in results if item.get("status") == "exported"),
        "errors": sum(1 for item in results if item.get("status") == "error"),
        "results": results,
    }
    REPORT.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 1 if report["errors"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
