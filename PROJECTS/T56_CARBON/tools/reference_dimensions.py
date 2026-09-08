#!/usr/bin/env python3
"""Offline ANSUR reference import and T56 dimensional comparison, never adoption."""

from __future__ import annotations

import argparse
import csv
from hashlib import sha256
import io
import json
import math
from pathlib import Path
import statistics
import sys

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from skeleton.canon import instantiate_sophy_canon

PROJECT = ROOT / "PROJECTS/T56_CARBON"
REFERENCES = PROJECT / "references"


def load(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def digest(value):
    return sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def number(value, label, positive=True):
    if isinstance(value, bool):
        raise ValueError(f"{label}: booleans are not measurements")
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{label}: invalid number") from exc
    if not math.isfinite(result) or (positive and result <= 0):
        raise ValueError(f"{label}: expected a finite {'positive ' if positive else ''}number")
    return result


def import_csv(raw: bytes, catalog: dict) -> dict:
    """Project only age, row provenance and selected measured lengths from pinned bytes."""
    source = catalog["ansur"]
    if sha256(raw).hexdigest() != source["csv_sha256"]:
        raise ValueError("source CSV SHA-256 mismatch; review a new source revision before import")
    if source["length_units"] != "mm":
        raise ValueError("ANSUR import requires documented millimetre units")
    reader = csv.DictReader(io.StringIO(raw.decode("cp1252")))
    measures = sorted(catalog["measurements"])
    required = ["Gender", "Age", "SubjectId", *measures]
    if reader.fieldnames is None or len(reader.fieldnames) != len(set(reader.fieldnames)) or not set(required) <= set(reader.fieldnames):
        raise ValueError("missing or duplicate source CSV columns")
    rows, seen = [], set()
    for line, record in enumerate(reader, 2):
        if record["Gender"] != "Female" or record["SubjectId"] in seen:
            raise ValueError(f"source row {line}: incorrect population or duplicate subject")
        seen.add(record["SubjectId"])
        age = number(record["Age"], "Age")
        if not age.is_integer():
            raise ValueError(f"source row {line}: age must be an integer")
        values = [number(record[key], f"row {line}/{key}") for key in measures]
        rows.append([line, int(age), *values])
    if len(rows) != source["expected_rows"]:
        raise ValueError("source row count mismatch")
    for index, key in enumerate(measures, 2):
        observed = statistics.mean(row[index] for row in rows)
        # Report means are rounded to 0.01 cm; 0.1 mm covers final-digit rounding.
        if abs(observed - catalog["measurements"][key]["female_mean_cm"] * 10) > 0.1:
            raise ValueError(f"{key}: public data mean disagrees with report (check units or source)")
    return {"schema_version": "1.0.0", "source_id": source["source_id"],
            "source_csv_sha256": source["csv_sha256"], "units": "mm",
            "columns": ["source_csv_line", "age_years", *measures], "rows": rows}


def validate_snapshot(snapshot, catalog):
    if digest(snapshot) != catalog["ansur"]["projected_snapshot_sha256"]:
        raise ValueError("projected snapshot SHA-256 mismatch; reimport the pinned source")
    expected = ["source_csv_line", "age_years", *sorted(catalog["measurements"])]
    if snapshot.get("columns") != expected or snapshot.get("units") != "mm":
        raise ValueError("snapshot columns or units differ from catalog")
    if snapshot.get("source_id") != catalog["ansur"]["source_id"] or snapshot.get("source_csv_sha256") != catalog["ansur"]["csv_sha256"]:
        raise ValueError("snapshot is not bound to the selected source")
    rows = snapshot.get("rows")
    if not isinstance(rows, list) or len(rows) != catalog["ansur"]["expected_rows"]:
        raise ValueError("snapshot row count mismatch")
    seen = set()
    for row in rows:
        if not isinstance(row, list) or len(row) != len(expected):
            raise ValueError("invalid snapshot row shape")
        if type(row[0]) is not int or row[0] < 2 or row[0] in seen or type(row[1]) is not int:
            raise ValueError("invalid/duplicate row provenance or age")
        seen.add(row[0])
        for value in row[1:]:
            if type(value) not in (int, float):
                raise ValueError("snapshot measurements must be JSON numbers")
            number(value, "snapshot value")
    return [dict(zip(expected, row)) for row in rows]


def quantile(values, probability):
    ordered = sorted(values)
    rank = (len(ordered) - 1) * probability
    lo = math.floor(rank)
    hi = math.ceil(rank)
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (rank - lo)


def select(rows, height, ages, stature_band, span_band):
    return [row for row in rows if ages[0] <= row["age_years"] <= ages[1]
            and abs(row["stature"] - height) <= stature_band
            and abs(row["span"] - height) <= span_band]


def compile_specs(snapshot, catalog, policy, profile, legacy):
    rows = validate_snapshot(snapshot, catalog)
    reference = profile["canon_reference"]
    height = number(reference["height_mm"], "canon height")
    canon = instantiate_sophy_canon(height, canon_version=reference["canon_version"])
    if profile["height_mm"] != height or profile["arm_span_mm"] != canon.arm_span_mm:
        raise ValueError("profile disagrees with canon")
    ages = policy["age_years"]
    if not isinstance(ages, list) or len(ages) != 2 or any(type(a) is not int for a in ages) or not 18 <= ages[0] <= ages[1]:
        raise ValueError("age range must contain two ordered adult integer ages")
    sb = number(policy["stature_band_mm"], "stature band")
    ab = number(policy["span_band_mm"], "span band")
    minimum = policy["minimum_cohort_size"]
    if type(minimum) is not int or minimum < 2:
        raise ValueError("minimum cohort size must be an integer of at least two")
    cohort = select(rows, height, ages, sb, ab)
    if len(cohort) < minimum:
        raise ValueError(f"insufficient matched cohort: {len(cohort)} < {minimum}; no automatic widening")
    measures = sorted(catalog["measurements"])
    medians = {key: statistics.median(row[key] for row in cohort) for key in measures}
    # A real row is provided alongside marginal summaries; it is not a canon proposal.
    representative = min(cohort, key=lambda row: (
        sum(abs(row[key] - medians[key]) / medians[key] for key in measures), row["source_csv_line"]))
    scale = height / number(legacy["height_mm"], "legacy height")
    dimensions = []
    for key in measures:
        definition = catalog["measurements"][key]
        target = definition["target"]
        legacy_value = number(legacy[target[0]][target[1]], "legacy reference") if target else None
        comparison = legacy_value * scale if target else None
        median = medians[key]
        dimensions.append({
            "measurement": key, "status": "reference_candidate_not_adopted",
            "reference_median_mm": median,
            "observed_p05_mm": round(quantile([row[key] for row in cohort], .05), 3),
            "observed_p95_mm": round(quantile([row[key] for row in cohort], .95), 3),
            "reference_ratio_to_H": median / height,
            "representative_row_mm": representative[key],
            "definition": definition["definition"], "mapping": definition["mapping"],
            "source_url": catalog["ansur"]["report_url"] + f"#page={definition['pdf_page']}",
            "source_section": definition["section"], "source_printed_page": definition["printed_page"],
            "legacy_target_path": target, "legacy_unscaled_mm": legacy_value,
            "legacy_at_target_height_mm": round(comparison, 3) if comparison else None,
            "delta_from_legacy_reference_mm": round(median - comparison, 3) if comparison else None,
            "delta_from_legacy_reference_percent": round(100 * (median / comparison - 1), 3) if comparison else None,
            "fabrication_value_mm": None, "canon_adopted": False,
        })
    historical = []
    for key, ratio in sorted(catalog["historical"]["ratios"].items()):
        numerator, denominator = ratio
        value = height * number(numerator, "ratio numerator") / number(denominator, "ratio denominator")
        historical.append({"measurement": key, "fraction_of_H": ratio, "historical_mm": value,
                           "empirical_median_mm": medians[key], "difference_mm": value - medians[key],
                           "source_url": catalog["historical"]["url"], "canon_adopted": False})
    sensitivity = []
    for band in policy["sensitivity_bands_mm"]:
        number(band, "sensitivity band")
        selected = select(rows, height, ages, band, band)
        enough = len(selected) >= minimum
        sensitivity.append({"band_mm_each": band, "n": len(selected), "sufficient": enough,
                            "medians_mm": {key: statistics.median(row[key] for row in selected) for key in measures} if enough else None})
    return {"artifact": "T56_REFERENCE_DIMENSION_SPECS", "schema_version": "1.0.0",
            "status": "reference_candidates_require_endpoint_review_and_canon_adoption",
            "fabrication_ready": False, "canon_reference": {"height_mm": height, "version": canon.canon_version, "law_hash": canon.canon_law_hash},
            "provenance": {"source_csv_sha256": catalog["ansur"]["csv_sha256"],
                           "snapshot_sha256": digest(snapshot), "catalog_sha256": digest(catalog),
                           "policy_sha256": digest(policy), "profile_sha256": digest(profile), "legacy_sha256": digest(legacy)},
            "selection": {"policy": policy, "source_n": len(rows), "cohort_n": len(cohort),
                          "source_csv_lines": sorted(row["source_csv_line"] for row in cohort)},
            "dimensions": dimensions, "representative_observed_row": representative,
            "representative_span_minus_H_mm": representative["span"] - height,
            "sensitivity": sensitivity, "historical_comparison": historical,
            "supporting_references": catalog.get("supporting_references", []),
            "unresolved_targets": catalog["unresolved_targets"]}


def markdown(report):
    lines = ["# T-5.6 reference dimension comparison", "", "**Reference candidates only; not fabrication release or canon adoption.**", "",
             f"H = {report['canon_reference']['height_mm']} mm; matched sample n = {report['selection']['cohort_n']} of {report['selection']['source_n']}.", "",
             "Measured values are unscaled. Legacy values alone are scaled from REDWOOD for comparison. Endpoint mismatches remain explicit.", "",
             "| Measure | Median mm | Observed p05–p95 mm | Scaled legacy mm | Delta mm | Mapping |",
             "| --- | ---: | ---: | ---: | ---: | --- |"]
    for row in report["dimensions"]:
        legacy = row['legacy_at_target_height_mm'] if row['legacy_at_target_height_mm'] is not None else '—'
        delta = row['delta_from_legacy_reference_mm'] if row['delta_from_legacy_reference_mm'] is not None else '—'
        lines.append(f"| [{row['measurement']}]({row['source_url']}) | {row['reference_median_mm']:g} | {row['observed_p05_mm']:g}–{row['observed_p95_mm']:g} | {legacy} | {delta} | {row['mapping']} |")
    lines += ["", "The percentile columns describe this selected sample; they are not uncertainty bounds or manufacturing tolerances. Marginal medians do not constitute one observed body. The JSON includes an actual representative row and its nonzero span residual; no body is morphed to fit H.", "", "## Historical comparison", ""]
    for row in report["historical_comparison"]:
        lines.append(f"- [{row['measurement']}]({row['source_url']}): historical {row['historical_mm']:.2f} mm versus measured-cohort median {row['empirical_median_mm']:.2f} mm. Neither is silently adopted.")
    lines += ["", "## Selection sensitivity", ""]
    for row in report["sensitivity"]:
        lines.append(f"- ±{row['band_mm_each']} mm on both stature and span: n={row['n']}; sufficient={row['sufficient']}.")
    lines += ["", "## Supporting references", ""]
    for source in report["supporting_references"]:
        lines.append(f"- [{source['title']}]({source['url']}), sections {', '.join(source['locators'])}: {source['role']}.")
    lines += ["", "## Still unresolved", ""]
    lines += [f"- {key}: {value}" for key, value in report["unresolved_targets"].items()]
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--import-csv", type=Path, help="import exact pinned public CSV to the specified output; no network")
    parser.add_argument("--snapshot", type=Path, default=REFERENCES / "ansur_female_dimensions.json")
    parser.add_argument("--catalog", type=Path, default=REFERENCES / "dimensional_sources.json")
    parser.add_argument("--policy", type=Path, default=REFERENCES / "dimensional_selection.json")
    parser.add_argument("--profile", type=Path, default=PROJECT / "profiles/t56_domestic_frame.json")
    parser.add_argument("--legacy", type=Path, default=ROOT / "PROJECTS/REDWOOD/profiles/adult_female_21_28.json")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--markdown", type=Path)
    args = parser.parse_args()
    try:
        catalog = load(args.catalog)
        if args.import_csv:
            result = import_csv(args.import_csv.read_bytes(), catalog)
            # One numeric row per line keeps the projected dataset compact and reviewable.
            header = {k: v for k, v in result.items() if k != "rows"}
            content = json.dumps(header, sort_keys=True, indent=2)[:-2] + ',\n  "rows": [\n'
            content += ",\n".join("    " + json.dumps(row, separators=(",", ":")) for row in result["rows"]) + "\n  ]\n}\n"
        else:
            result = compile_specs(load(args.snapshot), catalog, load(args.policy), load(args.profile), load(args.legacy))
            content = json.dumps(result, sort_keys=True, indent=2, allow_nan=False) + "\n"
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(content, encoding="utf-8")
        if args.markdown and not args.import_csv:
            args.markdown.parent.mkdir(parents=True, exist_ok=True)
            args.markdown.write_text(markdown(result), encoding="utf-8")
    except (ValueError, OSError, KeyError, TypeError) as exc:
        print(f"reference dimensions: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
