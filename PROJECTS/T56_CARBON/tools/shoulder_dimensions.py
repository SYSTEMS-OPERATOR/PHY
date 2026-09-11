#!/usr/bin/env python3
"""Offline dummy-member dimensional review. Never releases fabrication geometry."""
import argparse
import hashlib
import html
import json
import math
from pathlib import Path

POINTS = ("shoulder_output", "dummy_elbow_station")
AXIAL = ("shoulder_offset", "elbow_offset", "shoulder_insertion", "elbow_insertion")
FIELDS = POINTS + AXIAL
MODEL = "straight_collinear_dummy_member_v1"


def number(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def nonblank(value):
    return isinstance(value, str) and bool(value.strip())


def review(data):
    """Validate input and resolve nominal centerline and conservative length bounds."""
    errors, blockers = [], []
    result = {"status": "blocked", "fabrication_ready": False,
              "canon_adopted": False, "errors": errors, "blockers": blockers}
    if not isinstance(data, dict):
        errors.append("input must be an object")
        return result
    if set(data) != {"model", "frame_id", "pose_id", "datum_drawing", "records"}:
        errors.append("input keys must be model, frame_id, pose_id, datum_drawing, records")
    if data.get("model") != MODEL:
        errors.append("only the straight collinear dummy-member model is supported")
    if data.get("frame_id") != "FRAME_T56_THORAX":
        errors.append("all points must be in FRAME_T56_THORAX")
    for key in ("pose_id", "datum_drawing"):
        if data.get(key) is None:
            blockers.append(f"{key} remains open")
        elif not nonblank(data[key]):
            errors.append(f"{key} must be a nonblank string")
    records = data.get("records")
    if not isinstance(records, dict) or set(records) != set(FIELDS):
        errors.append("records must contain exactly the six named mechanical inputs")
        return result
    for name in FIELDS:
        row = records[name]
        if not isinstance(row, dict) or set(row) != {"value", "units", "tolerance_mm", "evidence"}:
            errors.append(f"{name}: expected value, units, tolerance_mm, evidence")
            continue
        value, tol = row["value"], row["tolerance_mm"]
        if row["units"] != "mm":
            errors.append(f"{name}: units must be mm")
        if value is None:
            blockers.append(f"{name}: measurement/design dimension remains open")
        elif name in POINTS:
            if not isinstance(value, list) or len(value) != 3 or not all(number(v) for v in value):
                errors.append(f"{name}: expected three finite numeric coordinates")
        elif not number(value) or value < 0:
            errors.append(f"{name}: expected nonnegative finite axial dimension")
        if tol is None:
            blockers.append(f"{name}: tolerance remains open")
        elif not number(tol) or tol <= 0:
            errors.append(f"{name}: tolerance must be positive and finite")
        if row["evidence"] is None:
            blockers.append(f"{name}: drawing/measurement evidence remains open")
        elif not nonblank(row["evidence"]):
            errors.append(f"{name}: evidence must be a nonblank string")
        if name in AXIAL and number(value) and number(tol) and value < tol:
            errors.append(f"{name}: tolerance interval extends below zero")
    if errors or blockers:
        return result
    s, e = (records[n]["value"] for n in POINTS)
    delta = [b - a for a, b in zip(s, e)]
    length = math.hypot(*delta)
    endpoint_bound = sum(records[n]["tolerance_mm"] for n in POINTS)
    os, oe, ins, ine = (records[n]["value"] for n in AXIAL)
    gap = length - os - oe
    gap_bound = endpoint_bound + sum(records[n]["tolerance_mm"] for n in AXIAL[:2])
    cut = gap + ins + ine
    cut_bound = gap_bound + sum(records[n]["tolerance_mm"] for n in AXIAL[2:])
    if not all(number(v) for v in (length, gap, cut, endpoint_bound, gap_bound, cut_bound)):
        errors.append("dimensional arithmetic overflow")
        return result
    if length <= endpoint_bound or gap <= gap_bound:
        errors.append("endpoints or seats overlap within the specified tolerance bounds")
        return result
    u = [v / length for v in delta]
    def along(distance):
        return [a + distance * b for a, b in zip(s, u)]
    points = {"S": s, "E": e, "seat_S": along(os), "seat_E": along(length - oe),
              "cut_S": along(os - ins), "cut_E": along(length - oe + ine)}
    if not all(number(v) for p in points.values() for v in p):
        errors.append("derived coordinate overflow")
        return result
    result.update(status="dimensional_review_only", frame_id=data["frame_id"],
                  pose_id=data["pose_id"], points_mm=points, axis_unit=u,
                  dimensions_mm={"effective_length": length, "seat_gap": gap,
                                 "stock_cut_length": cut},
                  worst_case_bounds_mm={"effective_length": endpoint_bound,
                                        "seat_gap": gap_bound, "stock_cut_length": cut_bound},
                  formula="stock_cut_length = |E-S| - oS - oE + iS + iE")
    return result


def svg(result):
    """Three orthographic centerline views; no implied section or hole geometry."""
    if result["status"] != "dimensional_review_only":
        raise ValueError("cannot draw unresolved geometry")
    pts = result["points_mm"]
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" width="1100" height="720" viewBox="0 0 1100 720">',
             '<rect width="1100" height="720" fill="white"/>',
             '<g font-family="monospace" fill="#172230">',
             '<text x="32" y="35" font-size="20">T56 DUMMY MEMBER — DIMENSION REVIEW ONLY</text>',
             '<text x="32" y="62" font-size="14">NOT FOR FABRICATION • centerlines only • no section, holes or load rating</text>']
    for index, (i, j, title) in enumerate(((0, 2, "XZ FRONT"), (1, 2, "YZ SIDE"), (0, 1, "XY TOP"))):
        left = 30 + index * 360
        lowx, highx = min(p[i] for p in pts.values()), max(p[i] for p in pts.values())
        lowy, highy = min(p[j] for p in pts.values()), max(p[j] for p in pts.values())
        scale = 230 / max(highx - lowx, highy - lowy, 1)
        def xy(p):
            return (left + 160 + (p[i] - (lowx + highx) / 2) * scale,
                    230 - (p[j] - (lowy + highy) / 2) * scale)
        parts.append(f'<text x="{left}" y="100" font-size="16">{title} (+axes right/up)</text>')
        for a, b, color, width in (("S", "E", "#8292a1", 2), ("cut_S", "cut_E", "#156f88", 5)):
            x1, y1 = xy(pts[a]); x2, y2 = xy(pts[b])
            parts.append(f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{color}" stroke-width="{width}"/>')
        for k, (name, p) in enumerate(pts.items()):
            x, y = xy(p)
            parts.append(f'<circle cx="{x}" cy="{y}" r="4" fill="#172230"/>')
            # Numbered point key avoids overlapping labels for close socket seats.
            parts.append(f'<text x="{left}" y="{390+k*21}" font-size="12">{name}: ({p[i]:.3f}, {p[j]:.3f}) mm</text>')
    for k, (name, value) in enumerate(result["dimensions_mm"].items()):
        tol = result["worst_case_bounds_mm"][name]
        parts.append(f'<text x="32" y="{545+k*28}" font-size="17">{name}: {value:.3f} ± {tol:.3f} mm (worst case)</text>')
    parts.append(f'<text x="32" y="645" font-size="13">Input SHA-256: {html.escape(result.get("input_sha256", "not serialized"))}</text>')
    parts.append('<text x="32" y="675" font-size="13">Views fit to page independently; coordinate tables control. No printed drawing scale.</text></g></svg>')
    return "\n".join(parts) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()
    try:
        raw = args.input.read_bytes()
        data = json.loads(raw)
        # Also catches exponent overflow such as 1e309, not just literal NaN.
        json.dumps(data, allow_nan=False)
        result = review(data)
    except (OSError, ValueError) as exc:
        parser.exit(1, f"Invalid input: {exc}\n")
    result["input_sha256"] = hashlib.sha256(raw).hexdigest()
    result["inputs"] = data
    args.output_dir.mkdir(parents=True, exist_ok=True)
    drawing = args.output_dir / "shoulder_member_review.svg"
    # Never leave a successful old drawing beside a newly blocked report.
    if drawing.exists():
        drawing.unlink()
    (args.output_dir / "shoulder_member_review.json").write_text(
        json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n")
    lines = ["# Shoulder member dimensional review", "", f"Status: **{result['status']}**",
             "", "Fabrication ready: **false**. Canon adoption: **false**.", "",
             f"Input SHA-256: `{result['input_sha256']}`", ""]
    if result["status"] == "dimensional_review_only":
        lines += ["| Dimension | Nominal mm | Worst-case ± mm |", "| --- | ---: | ---: |"]
        for name, value in result["dimensions_mm"].items():
            lines.append(f"| {name} | {value:.6f} | {result['worst_case_bounds_mm'][name]:.6f} |")
        lines += ["", result["formula"], "", "Centerline review only; no cross-section, drill schedule or load approval."]
        drawing.write_text(svg(result))
    else:
        lines += [f"- {message}" for message in result["errors"] + result["blockers"]]
    (args.output_dir / "SHOULDER_MEMBER_REVIEW.md").write_text("\n".join(lines) + "\n")
    print(result["status"])
    return 1 if result["errors"] else 2 if result["blockers"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
