"""Read-only, deterministic Studio projection of existing engineering evidence.

Use fresh canonical records and the existing auditors, never cached reports or
display-mesh measurements. The explicit baseline detects stale review packages;
it is a custody pin, not another anatomical or fabrication authority.
"""
from __future__ import annotations

from copy import deepcopy
import math
from pathlib import Path
import re
import warnings

from skeleton.bones import load_field
from skeleton.canon import instantiate_sophy_canon
from skeleton.exporters.exporter_agent import json_bytes, sha256
from skeleton.validation.dimension_evidence import (
    CATALOG_PATH, _pinned_file, expansion_report, load_evidence, read_json,
)
from skeleton.validation.engineering_audit import engineering_audit
from skeleton.relationships import build_relationship_snapshot

ROOT = Path(__file__).resolve().parents[2]
BASELINE_PATH = Path("studio/evidence_baseline.json")
CONDITIONS = ("unsuited", "suited_unpressurized", "suited_pressurized")


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def validate_nasa(catalog):
    """Reject accidental engineering adoption; retain source signs and issues."""
    if (catalog.get("catalog_version") != "1.0.0"
            or catalog.get("authority") != "reference_comparison_only"
            or catalog.get("canon_effect") != "none"
            or catalog.get("fabrication_released") is not False):
        raise ValueError("NASA catalog must remain a reference comparison")
    sources = catalog["sources"]
    ids = [s["source_id"] for s in sources]
    if len(ids) != len(set(ids)) or not ids:
        raise ValueError("Ambiguous NASA sources")
    for source in sources:
        if not source["uri"].startswith("https://www.nasa.gov/"):
            raise ValueError("NASA source must retain its official locator")
    rows = catalog["ranges"]
    row_ids = [r["id"] for r in rows]
    if len(row_ids) != len(set(row_ids)) or not row_ids:
        raise ValueError("Ambiguous NASA motion rows")
    for row in rows:
        if (row["condition"] not in CONDITIONS or row["source_id"] not in ids
                or row["units"] != "deg"
                or any(row[key] is not None for key in (
                    "mechanical_axis", "mechanical_limit_deg", "combined_joint_feasibility"))):
            raise ValueError("NASA motion is not a mechanical constraint")
        ends = row["endpoints_as_printed_deg"]
        if not isinstance(ends, dict) or len(ends) != 2 or not all(_finite(v) for v in ends.values()):
            raise ValueError("Invalid printed NASA motion endpoints")
        if not row["sign_convention"] or not row["sex_scope"]:
            raise ValueError("Missing NASA scope or sign convention")
        if row["source_issue"] is not None and not isinstance(row["source_issue"], str):
            raise ValueError("Invalid NASA source issue")
    for row in catalog["anthropometry"]:
        cm, mm = row["values_cm"], row["values_mm"]
        if (row["source_id"] not in ids or row["fabrication_target_mm"] is not None
                or len(cm) != 2 or len(mm) != 2
                or not all(_finite(v) and v > 0 for v in cm + mm)
                or cm[0] > cm[1]
                or not all(math.isclose(a * 10, b, rel_tol=1e-12) for a, b in zip(cm, mm))):
            raise ValueError("Invalid NASA anthropometric comparison or unit conversion")
    return deepcopy(catalog)


def build_studio_evidence(parts, *, root=ROOT, field=None):
    """Project all 206 identities, measurement lineage and conditioned NASA data."""
    root = Path(root)
    baseline = read_json(root / BASELINE_PATH)
    if (baseline.get("schema_version") != "PHY_STUDIO_EVIDENCE_BASELINE_V1"
            or not re.fullmatch(r"[0-9a-f]{40}", baseline.get("engineering_revision", ""))):
        raise ValueError("Invalid Studio engineering baseline")
    catalog_path = _pinned_file(root, baseline["dimension_catalog"])
    if catalog_path != (root / CATALOG_PATH).resolve():
        raise ValueError("Studio dimension catalog authority differs")
    nasa_path = _pinned_file(root, baseline["nasa_catalog"])
    if nasa_path != (root / "references/nasa_positioning_ranges.json").resolve():
        raise ValueError("Studio NASA catalog authority differs")
    nasa = validate_nasa(read_json(nasa_path))
    if field is None:
        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", message="Metrics for .* not found in dataset", module="skeleton.base")
            field = load_field("female_21_baseline")
    engineering = engineering_audit(field, parts, root=root)
    source_files = {r["source"]["path"]: r["source"]["sha256"] for r in engineering["bones"]}
    observed = {
        "canonical_records_sha256": engineering["inputs"]["canonical_records_sha256"],
        "canonical_sources_sha256": sha256(json_bytes(source_files)),
        "display_parts_sha256": engineering["inputs"]["display_parts_sha256"],
    }
    if any(baseline.get(key) != value for key, value in observed.items()):
        raise ValueError("Stale Studio engineering baseline; review and repin changed evidence")
    catalog, _, _ = load_evidence(root)
    expansion = expansion_report(field, engineering, root=root, catalog=catalog)
    protocols = read_json(_pinned_file(root, catalog["measurement_protocols"]))
    facts = {sid: read_json(_pinned_file(root, source["native_snapshot"]))
             for sid, source in catalog["sources"].items()}
    candidates = {c["bone_id"]: c for c in catalog["candidates"]}
    bones = {}
    for row in engineering["bones"]:
        uid = row["bone_id"]
        candidate = candidates.get(uid)
        traces = {}
        if candidate:
            for key, mapping in candidate["fields"].items():
                sid = mapping["source_id"]
                native = next(r for r in facts[sid]["rows"] if r["parameter"] == mapping["parameter"])
                protocol_source = protocols["sources"][sid]
                traces[key] = {
                    "source_id": sid, "native_measurement": deepcopy(native),
                    "derivation": deepcopy(mapping),
                    "landmark_definition": protocol_source["fields"][key]["landmark_definition"],
                    "method_locator": protocol_source["source_locator"],
                    "uncertainty": protocol_source["uncertainty"],
                    "evidence_maturity": candidate["value_maturity"],
                    "validation_result": "custody_and_tabular_derivation_pass",
                    "geometry_qualified": False, "fabrication_qualified": False,
                }
        bones[uid] = {key: deepcopy(row[key]) for key in (
            "bone_id", "name", "region", "side", "source", "dimensions",
            "canonical_geometry", "anatomical_references", "record_validation", "evidence_needed")}
        bones[uid].update({
            "measurements": traces,
            "reference_declarations": deepcopy(row["references"]),
            "candidate_notes": candidate["notes"] if candidate else [],
            "embodiment_fit": candidate["embodiment_fit"] if candidate else None,
        })
    canon = instantiate_sophy_canon(1676.4)
    canon_data = canon.to_dict()
    if (canon.canon_version != catalog["canonical_reference"]["version"]
            or canon.standing_height_mm != catalog["canonical_reference"]["height_mm"]
            or canon.arm_span_mm != catalog["canonical_reference"]["span_mm"]):
        raise ValueError("Studio canon comparison differs from the current kernel")
    relationships = build_relationship_snapshot(field, root=root)
    inputs = {
        **observed,
        "inventory_sha256": engineering["inputs"]["inventory_sha256"],
        "baseline_sha256": sha256((root / BASELINE_PATH).read_bytes()),
        "dimension_catalog_sha256": baseline["dimension_catalog"]["sha256"],
        "measurement_protocols_sha256": catalog["measurement_protocols"]["sha256"],
        "source_snapshots": {sid: s["native_snapshot"] for sid, s in catalog["sources"].items()},
        "nasa_catalog_sha256": baseline["nasa_catalog"]["sha256"],
        "canon_sha256": sha256(json_bytes(canon_data)),
        "relationship_graph_sha256": sha256(json_bytes(relationships)),
    }
    evidence = {
        "version": "PHY_STUDIO_EVIDENCE_V1", "engineering_revision": baseline["engineering_revision"],
        "engineering_revision_time": baseline["engineering_revision_time"],
        "inputs": inputs, "input_sha256": sha256(json_bytes(inputs)),
        "canonical_reference": catalog["canonical_reference"],
        "metrics": engineering["metrics"], "qualification": engineering["qualification"],
        "dimension_expansion": {k: v for k, v in expansion.items() if k not in ("rows", "inputs_sha256", "source_facts")},
        "bones": bones, "sources": catalog["sources"], "nasa": nasa,
        "relationships": relationships,
        "nasa_counts": {"conditions": {c: sum(r["condition"] == c for r in nasa["ranges"]) for c in CONDITIONS},
                        "source_issues": sum(r["source_issue"] is not None for r in nasa["ranges"]),
                        "anthropometry": len(nasa["anthropometry"])},
        "fabrication_released": False,
    }
    json_bytes(evidence)
    return evidence


def studio_build_stamp(evidence, root=ROOT, native_references=None):
    """Fingerprint the shipped viewer sources, without a self-referential HEAD."""
    paths = ("bin/export_phy_studio.py", "skeleton/visualization/studio_evidence.py",
             "studio/src/app.js", "studio/src/evidence.js", "studio/src/index.html", "studio/src/part_roles.js",
             "studio/app.bundle.js", "studio/build.mjs", "studio/package-lock.json",
             "skeleton/visualization/studio_references.py", "studio/reference_lock.json", "studio/src/native_reference.js",
             "studio/src/original_review.js", "studio/src/original_review_ui.js",
             "studio/src/relationships.js", "skeleton/identity.py", "skeleton/relationships.py",
             "skeleton/relationship_catalog.py", "skeleton/schema/relationship_graph.schema.json")
    inputs = {path: sha256((Path(root) / path).read_bytes()) for path in paths}
    inputs["engineering_evidence"] = sha256(json_bytes(evidence))
    inputs["native_reference_evidence"] = sha256(json_bytes(native_references))
    return {"engineering_revision": evidence["engineering_revision"],
            "evidence_input_sha256": evidence["input_sha256"],
            "package_input_sha256": sha256(json_bytes(inputs)), "inputs": inputs}
