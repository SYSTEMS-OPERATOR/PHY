"""Trace adult identities across records and display proxies without qualification.

This is an observation report, not a new validation or release gate. Counts of
declared data never certify osteometry, anatomical surfaces or working joints.
"""
from __future__ import annotations

from copy import deepcopy
from pathlib import Path

from skeleton.anatomical_inventory import ROOT, INVENTORY_VERSION, expected_bones
from skeleton.exporters.exporter_agent import json_bytes, sha256
from skeleton.exporters.identity import ExportIdentity
from skeleton.exporters.publication import publish_files
from skeleton.field import SkeletonField
from skeleton.validation.validator_agent import ValidatorAgent, finite_number
from skeleton.visualization.bone_equivalence import bone_audit


AUDIT_VERSION = "1.0.0"
METRIC_MEANINGS = {
    "individual_display_identity": "Individual provisional proxy, excluding hardware and grouped forms.",
    "source_module_present": "Local module declares the expected identity; not evidence of measured data.",
    "canonical_record_present": "Supplied canonical snapshot contains the expected identity.",
    "declared_dimensions_complete": "Nonempty declared dimensions are all finite, positive numbers; not dimensional fidelity.",
    "dataset_binding_present": "Selected dataset contains an object at the record's declared binding key; not evidence qualification.",
    "reference_declaration_present": "Record declares at least one nonempty reference string; citations are not verified.",
    "dimensional_primitive_available": "Canonical geometry has dimensional_estimate status; not an anatomical surface.",
    "declared_anatomical_references_resolved": "At least one declared reference resolves, all declarations resolve and no parent cycle involves this identity; not articulation qualification.",
    "record_validation_pass": "Existing ValidatorAgent reports no per-record blocker; independent of whole-model and physical release gates.",
}


def engineering_audit(skeleton: SkeletonField, parts, *, root: Path = ROOT):
    """Inspect a supplied canonical snapshot and reference-body proxy snapshot.

    Refuse ambiguous IDs, unsupported identities and non-serializable records.
    Missing records and null measurements are normal reportable gaps. Source
    modules are inspected statically; no dimensions are taken from proxy meshes.
    """
    root = Path(root)
    snapshot = deepcopy(skeleton)
    parts = deepcopy(list(parts))
    display = bone_audit(parts, root=root)
    inventory = expected_bones()
    wanted = {row["bone_id"] for row in inventory}
    bones = {}
    for bone in snapshot.bones.values():
        uid = bone.unique_id
        if not isinstance(uid, str) or uid not in wanted:
            raise ValueError(f"Unknown canonical adult bone ID: {uid!r}")
        if uid in bones:
            raise ValueError(f"Duplicate canonical adult bone ID: {uid}")
        bones[uid] = bone

    records = []
    for uid, bone in sorted(bones.items()):
        try:
            record = bone.to_fabrication_record()
            json_bytes(record)
        except (TypeError, ValueError, AttributeError, KeyError, OverflowError) as error:
            raise ValueError(f"Cannot audit canonical record {uid}: {error}") from error
        records.append(record)
    record_by_id = {record["unique_id"]: record for record in records}
    validation = ValidatorAgent(snapshot).run()
    identity = ExportIdentity(records)
    if identity.fatal:
        raise ValueError("Canonical identity snapshot is ambiguous")
    blockers = {uid: [] for uid in bones}
    for category, issues in validation.items():
        if not isinstance(issues, list):
            continue
        for issue in issues:
            uid = issue.get("bone") if isinstance(issue, dict) else issue
            if uid in blockers and category not in blockers[uid]:
                blockers[uid].append(category)
    cycle_ids = {uid for cycle in identity.report["parent_cycles"] for uid in cycle}
    reference_issues = {uid: [] for uid in bones}
    for issue in identity.report["unresolved_references"]:
        reference_issues[issue["bone_id"]].append(issue)
    display_by_id = {row["bone_id"]: row for row in display["bones"]}

    datasets = {}
    rows = []
    for anatomy in inventory:
        uid = anatomy["bone_id"]
        proxy = display_by_id[uid]
        bone = bones.get(uid)
        record = record_by_id.get(uid)
        source = proxy["source_record"]
        row = {
            **anatomy,
            "source": {"status": "present" if source else "missing", "path": source,
                       "sha256": sha256((root / source).read_bytes()) if source else None},
            "display": {key: proxy[key] for key in (
                "representation", "mesh_ids", "geometry_inputs", "dimensional_fidelity", "physical_evidence")},
            "canonical_record": {"status": "present" if record else "missing",
                                 "revision": record["revision"] if record else None},
            "dimensions": {"status": "missing_record", "values_mm": None,
                           "known_keys": [], "unknown_keys": [], "invalid_keys": [],
                           "metric_source_declarations": {}},
            "dataset": {"status": "missing_record", "key": None, "sha256": None},
            "references": {"status": "missing_record", "declarations": []},
            "canonical_geometry": record["geometry"] if record else None,
            "anatomical_references": {"status": "missing_record", "parent": None,
                                      "children": [], "parent_cycle": False, "issues": []},
            "record_validation": {"status": "missing_record", "blocker_categories": []},
            "evidence_needed": [],
        }
        if record:
            values = record["dimensions"]
            known = sorted(key for key, value in values.items() if finite_number(value) and value > 0)
            unknown = sorted(key for key, value in values.items() if value is None)
            invalid = sorted(set(values) - set(known) - set(unknown))
            complete = bool(values) and len(known) == len(values)
            row["dimensions"] = {
                "status": "complete" if complete else "partial" if known else "unknown",
                "values_mm": values, "known_keys": known, "unknown_keys": unknown,
                "invalid_keys": invalid,
                "metric_source_declarations": deepcopy(bone.metric_sources),
            }
            if bone.dataset is not None:
                digest = sha256(json_bytes(bone.dataset))
                datasets.setdefault(digest, []).append(uid)
                bound = (isinstance(bone.dataset, dict) and isinstance(bone.dataset_key, str)
                         and isinstance(bone.dataset.get(bone.dataset_key), dict))
                row["dataset"] = {"status": "bound" if bound else "unbound",
                                  "key": bone.dataset_key, "sha256": digest}
            else:
                row["dataset"]["status"] = "not_selected"
            # Inspect source types before to_fabrication_record's legacy list()
            # conversion can turn a malformed string into apparent citations.
            refs = bone.references
            declared = (isinstance(refs, list) and bool(refs)
                        and all(isinstance(ref, str) and ref.strip() for ref in refs))
            row["references"] = {
                "status": "declared" if declared else "missing" if isinstance(refs, list) and not refs else "invalid",
                "declarations": deepcopy(refs),
            }
            parent = identity.parent_references[uid]
            children = identity.adjacency[uid]
            declared_links = ([parent] if parent["status"] != "unspecified" else []) + children
            resolved = bool(declared_links) and all(link["bone_id"] is not None for link in declared_links)
            cycle = uid in cycle_ids
            row["anatomical_references"] = {
                "status": "unresolved" if reference_issues[uid] or cycle else "resolved" if resolved else "undeclared",
                "parent": parent, "children": children, "parent_cycle": cycle, "issues": reference_issues[uid],
            }
            row["record_validation"] = {
                "status": "blocked" if blockers[uid] else "pass",
                "blocker_categories": sorted(blockers[uid]),
            }
        needs = row["evidence_needed"]
        if not source:
            needs.append("canonical_source_module")
        if record is None:
            needs.append("canonical_record_in_snapshot")
        else:
            if row["dimensions"]["status"] != "complete":
                needs.append("supported_positive_dimensions")
            if row["dataset"]["status"] != "bound":
                needs.append("explicit_dataset_binding_or_documented_source_basis")
            if row["references"]["status"] != "declared":
                needs.append("traceable_reference_declarations")
            if row["anatomical_references"]["status"] != "resolved":
                needs.append("review_anatomical_relationship_declarations")
        # These stages have no qualification contract in the current reference
        # pipeline. A populated dimension, primitive or interface cannot pass them.
        needs.extend(("dimensional_evidence_review", "qualified_anatomical_geometry",
                      "qualified_articulation_and_transforms", "physical_fabrication_evidence"))
        rows.append(row)

    predicates = {
        "individual_display_identity": lambda r: r["display"]["representation"] == "individual_project_proxy",
        "source_module_present": lambda r: r["source"]["status"] == "present",
        "canonical_record_present": lambda r: r["canonical_record"]["status"] == "present",
        "declared_dimensions_complete": lambda r: r["dimensions"]["status"] == "complete",
        "dataset_binding_present": lambda r: r["dataset"]["status"] == "bound",
        "reference_declaration_present": lambda r: r["references"]["status"] == "declared",
        "dimensional_primitive_available": lambda r: bool(r["canonical_geometry"]) and r["canonical_geometry"].get("status") == "dimensional_estimate",
        "declared_anatomical_references_resolved": lambda r: r["anatomical_references"]["status"] == "resolved",
        "record_validation_pass": lambda r: r["record_validation"]["status"] == "pass",
    }

    def metrics(subset):
        return {key: {"count": sum(bool(predicate(row)) for row in subset), "total": len(subset),
                      "meaning": METRIC_MEANINGS[key]} for key, predicate in predicates.items()}

    report = {
        "audit_version": AUDIT_VERSION, "inventory_version": INVENTORY_VERSION,
        "scope": "adult_reference_identity_and_engineering_evidence",
        "inputs": {
            "inventory_sha256": sha256(json_bytes(inventory)),
            "canonical_records_sha256": sha256(json_bytes(records)),
            "display_parts_sha256": sha256(json_bytes(sorted(parts, key=lambda part: part["id"]))),
            "datasets": [{"sha256": digest, "bone_ids": sorted(ids)} for digest, ids in sorted(datasets.items())],
        },
        "metrics": metrics(rows),
        "regions": [{"region": region, "metrics": metrics([row for row in rows if row["region"] == region])}
                    for region in sorted({row["region"] for row in rows})],
        "missing_source_records": display["missing_source_records"],
        "missing_canonical_records": sorted(wanted - set(bones)),
        "record_validation": validation,
        "identity_validation": identity.report,
        "qualification": {
            "dimensional_fidelity": "unverified", "anatomical_geometry": "unqualified",
            "articulation": "unqualified", "mechanical_transforms": "unresolved",
            "physical_evidence": "unmeasured", "fabrication_released": False,
            "basis": "No independent reference-body qualification contract or physical acceptance evidence is evaluated by this audit.",
        },
        "bones": rows,
    }
    json_bytes(report)
    return report


def report_markdown(report):
    """Readable totals and a complete crosswalk, without a composite score."""
    def cell(value):
        return str(value).replace("|", "\\|").replace("\n", " ").replace("\r", " ")

    lines = ["# PHY engineering evidence audit", "",
             "Observation scope: adult reference identities, canonical records and provisional Studio proxies. A0-R1 remains separate.",
             "Declared data completeness is not anatomical or mechanical qualification. Fabrication release remains false.", "",
             "| Observation | Count / adult identities | Meaning |", "| --- | ---: | --- |"]
    for key, metric in report["metrics"].items():
        lines.append(f"| {key} | {metric['count']} / {metric['total']} | {metric['meaning']} |")
    lines += ["", "## Regional observations", "", "| Region | Identities | Records | Complete declared dimensions | Dataset bindings | Primitive estimates |",
              "| --- | ---: | ---: | ---: | ---: | ---: |"]
    for region in report["regions"]:
        m = region["metrics"]
        lines.append(f"| {region['region']} | {m['canonical_record_present']['total']} | {m['canonical_record_present']['count']} | {m['declared_dimensions_complete']['count']} | {m['dataset_binding_present']['count']} | {m['dimensional_primitive_available']['count']} |")
    lines += ["", "## Existing gates and qualification", "",
              f"Record validation pass: {report['record_validation']['summary']['pass']}.",
              f"Export identity validation ready: {report['identity_validation']['ready']}.",
              "These are the existing gates; this audit does not change export behavior or grant physical release.", ""]
    for key, value in report["qualification"].items():
        lines.append(f"- {key}: {value}")
    lines += ["", "## Every adult identity", "",
              "Detailed dimension values, dataset and source hashes, declarations, reference resolution and evidence needs are in engineering_audit.json.", "",
              "| Bone ID | Source | Canonical record | Display | Dimensions | Dataset | Geometry | Record blockers |",
              "| --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
    for row in report["bones"]:
        values = [row["bone_id"], row["source"]["path"] or "MISSING", row["canonical_record"]["status"],
                  row["display"]["representation"], row["dimensions"]["status"], row["dataset"]["status"],
                  (row["canonical_geometry"] or {}).get("status", "missing_record"),
                  ", ".join(row["record_validation"]["blocker_categories"]) or row["record_validation"]["status"]]
        lines.append("| " + " | ".join(cell(value) for value in values) + " |")
    lines += ["", "## Snapshot fingerprints", ""]
    for key in ("inventory_sha256", "canonical_records_sha256", "display_parts_sha256"):
        lines.append(f"- {key}: `{report['inputs'][key]}`")
    return "\n".join(lines) + "\n"


def write_reports(report, output_root: Path):
    """Publish a strict JSON/Markdown pair with hashes, using existing rollback."""
    directory = Path(output_root) / "reports"
    json_path = directory / "engineering_audit.json"
    md_path = directory / "engineering_audit.md"
    manifest_path = directory / "engineering_audit_manifest.json"
    payloads = {json_path: json_bytes(report), md_path: report_markdown(report).encode("utf-8")}
    payloads[manifest_path] = json_bytes({
        "audit_version": AUDIT_VERSION, "scope": report["scope"], "inputs": report["inputs"],
        "fabrication_released": False,
        "files": {path.name: {"sha256": sha256(content)} for path, content in payloads.items()},
    })
    publish_files(payloads, manifest_path)
    return {"json": str(json_path), "markdown": str(md_path), "manifest": str(manifest_path)}
