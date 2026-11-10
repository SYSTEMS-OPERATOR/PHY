"""Offline spine-data review, separate from geometry and physical qualification."""
from copy import deepcopy
from pathlib import Path

from skeleton.anatomical_inventory import ROOT, expected_bones, source_record_paths
from skeleton.exporters.exporter_agent import json_bytes, sha256
from skeleton.exporters.identity import ExportIdentity
from skeleton.exporters.publication import publish_files
from skeleton.identity import IdentityResolver
from skeleton.validation.dimension_evidence import load_evidence, read_json, _pinned_file


AUDIT_VERSION = "1.0.0"
FEATURES_PATH = "references/spine/features.json"
SOURCE_REVIEW_PATH = "references/spine/source_review.json"
SPINE_ORDER = tuple(f"BONE_{region}{level}" for region, count in (("C", 7), ("T", 12), ("L", 5))
                    for level in range(1, count + 1)) + ("BONE_SACRUM", "BONE_COCCYX")
WHOLE_BONE_FIELDS = ("length_mm", "width_mm", "thickness_mm")


def load_spine_features(root=ROOT):
    """Qualitative expectations for review; never inject geometry into BoneSpec."""
    catalog = read_json(Path(root) / FEATURES_PATH)
    if (catalog.get("schema_version") != "PHY_SPINE_FEATURES_V1"
            or catalog.get("authority") != "qualitative_reference_not_canon_or_mechanical_design"
            or catalog.get("anatomical_order") != list(SPINE_ORDER)
            or catalog.get("superior_reference") != "BONE_OCCIPITAL"):
        raise ValueError("Unsupported spine feature review contract")
    inventory = {row["bone_id"] for row in expected_bones() if row["region"] == "spine"}
    if set(SPINE_ORDER) != inventory:
        raise ValueError("Spine review must retain the 26 adult spine identities")
    by_id = {}
    for group in catalog["feature_sets"]:
        if (group["feature_geometry"] is not None or group["mechanical_definition"] is not None
                or not isinstance(group["features"], list) or not group["features"]
                or any(not isinstance(value, str) or not value.strip() for value in group["features"])
                or len(set(group["features"])) != len(group["features"])
                or not group["source_ids"] or not group["source_locator"]):
            raise ValueError("Feature evidence is qualitative and must retain source locators")
        for source_id in group["source_ids"]:
            source = catalog["sources"][source_id]
            if not source["url"].startswith("https://") or not source["scope"]:
                raise ValueError("Missing spine feature reference")
        for uid in group["bone_ids"]:
            if uid not in inventory or uid in by_id:
                raise ValueError("Unknown or duplicate spine feature identity")
            by_id[uid] = group
    if set(by_id) != inventory:
        raise ValueError("Incomplete spine feature review")
    return catalog, by_id


def audit_spine(field, *, root=ROOT):
    """Inspect supplied records without altering anatomy, geometry or runtime state.

    Consistency means agreement with the admitted tabular/qualitative contracts,
    not that unknown whole-bone dimensions or legacy dataset values are verified.
    Partial inputs remain partial; file presence never supplies a missing bone.
    """
    root = Path(root)
    catalog, feature_sets = load_spine_features(root)
    dimension_catalog, _, derived = load_evidence(root)
    review = read_json(root / SOURCE_REVIEW_PATH)
    if (review.get("schema_version") != "PHY_SPINE_SOURCE_REVIEW_V1"
            or review.get("scope") != "published_table_transcription_and_qualitative_feature_review_only"
            or review.get("geometry_qualified") is not False or review.get("fabrication_released") is not False
            or review.get("numerical_mismatches") != []):
        raise ValueError("Unsupported or inconsistent published-source review")
    reviewed_sources = set()
    for source in review["numerical_reviews"]:
        source_id = source["source_id"]
        if source_id in reviewed_sources:
            raise ValueError("Duplicate spine numerical review")
        reviewed_sources.add(source_id)
        facts = read_json(_pinned_file(root, source["native_snapshot"]))
        if (source["native_snapshot"] != dimension_catalog["sources"][source_id]["native_snapshot"]
                or source["rows_checked"] != len(facts["rows"])
                or source["admitted_rows"] != sum(row["admissible"] for row in facts["rows"])
                or source["quarantined_parameters"] != [row["parameter"] for row in facts["rows"] if not row["admissible"]]):
            raise ValueError("Spine source-review custody/count mismatch")
    if reviewed_sources != {"cervical_ct_2021", "thoracic_caliper_2011", "lumbar_ct_2022"}:
        raise ValueError("Incomplete spine numerical source review")

    snapshot = deepcopy(field)
    bones = list(snapshot.bones.values())
    resolver = IdentityResolver({"unique_id": bone.unique_id, "name": bone.name} for bone in bones)
    if resolver.fatal:
        raise ValueError("Spine snapshot has invalid or duplicate canonical identities")
    records = [bone.to_fabrication_record() for bone in sorted(bones, key=lambda b: b.unique_id)]
    json_bytes(records)  # Refuse non-JSON/nonfinite input before publication.
    identity = ExportIdentity(records)
    by_id = {bone.unique_id: bone for bone in bones}
    record_by_id = {row["unique_id"]: row for row in records}
    paths = source_record_paths(root)
    expected_parents = dict(zip(SPINE_ORDER, (catalog["superior_reference"],) + SPINE_ORDER[:-1]))
    rows, issues, declarations = [], [], []
    for uid in SPINE_ORDER:
        bone = by_id.get(uid)
        if bone is None:
            rows.append({"bone_id": uid, "record_status": "missing_record"})
            issues.append({"bone_id": uid, "check": "record_presence", "detail": "Not supplied"})
            continue
        record = record_by_id[uid]
        dims = record["dimensions"]
        group = feature_sets[uid]
        parent = identity.parent_references[uid]
        source_fields = derived.get(uid, {})
        checks = []
        if uid in derived:
            expected = {**dict.fromkeys(WHOLE_BONE_FIELDS), **source_fields}
            if dims != expected:
                checks.append(("dimension_semantics", "Dimensions differ from admitted feature spans and explicit whole-bone unknowns"))
        elif uid == "BONE_COCCYX" and dims != dict.fromkeys(WHOLE_BONE_FIELDS):
            checks.append(("dimension_semantics", "No quantitative coccyx dimensions have been admitted"))
        if bone.notable_features != group["features"]:
            checks.append(("features", "BoneSpec feature declarations differ from the cited qualitative review"))
        source_urls = [catalog["sources"][key]["url"] for key in group["source_ids"]]
        if any(url not in bone.references for url in source_urls):
            checks.append(("feature_references", "A qualitative feature reference is absent"))
        if parent["bone_id"] != expected_parents[uid]:
            checks.append(("anatomical_parent", "Expected superior anatomical reference: " + expected_parents[uid]))
        if any(uid in cycle for cycle in identity.report["parent_cycles"]):
            checks.append(("parent_cycle", "Declared parent cycle includes this identity"))
        issues.extend({"bone_id": uid, "check": check, "detail": detail} for check, detail in checks)
        locator = FEATURES_PATH + "#/feature_sets/" + str(catalog["feature_sets"].index(group))
        measurement_sources = {key: deepcopy(bone.metric_sources.get(key)) for key in source_fields}
        for key in source_fields:
            index = next(i for i, candidate in enumerate(dimension_catalog["candidates"]) if candidate["bone_id"] == uid)
            wanted = f"references/dim072/catalog.json#/candidates/{index}/fields/{key}"
            if measurement_sources[key] != wanted:
                issues.append({"bone_id": uid, "check": "measurement_provenance", "detail": "Missing or altered provenance: " + key})
        declarations.append({"record": record, "notable_features": bone.notable_features,
                             "metric_sources": bone.metric_sources})
        rows.append({
            "bone_id": uid, "record_status": "present", "dimensions_mm": dims,
            "admitted_feature_fields": sorted(source_fields), "measurement_provenance": measurement_sources,
            "whole_bone_unknown_fields": [key for key in WHOLE_BONE_FIELDS if dims.get(key) is None],
            "dimensional_status": "legacy_dataset_unverified" if uid == "BONE_SACRUM" else
                                  "partial_source_backed" if source_fields else "unknown",
            "dataset_key": bone.dataset_key,
            "anatomical_parent": parent, "expected_superior_reference": expected_parents[uid],
            "notable_features": deepcopy(bone.notable_features),
            "feature_evidence": {"catalog_locator": locator, "source_ids": group["source_ids"],
                                 "source_locator": group["source_locator"]},
            "feature_geometry": None, "mechanical_definition": None,
            "physics": record["physics"], "geometry": record["geometry"],
            "source_file": {"path": paths.get(uid), "sha256": sha256((root / paths[uid]).read_bytes()) if uid in paths else None},
            "reference_declarations": deepcopy(bone.references),
        })
    missing = [uid for uid in SPINE_ORDER if uid not in by_id]
    return {
        "audit_version": AUDIT_VERSION, "scope": "spine_dimensions_qualitative_features_and_declared_anatomical_parents",
        "canonical_reference": dimension_catalog["canonical_reference"],
        "counts": {"expected_spine_bones": 26, "present_spine_bones": 26 - len(missing),
                   "admitted_feature_fields_checked": sum(len(derived.get(uid, {})) for uid in SPINE_ORDER if uid in by_id),
                   "published_rows_reviewed": sum(row["rows_checked"] for row in review["numerical_reviews"]),
                   "consistency_issues": len(issues)},
        "complete_spine_snapshot": not missing, "consistency_pass": not issues,
        "all_dimensions_verified": False, "missing_record_ids": missing, "issues": issues, "bones": rows,
        "sources": catalog["sources"], "published_source_review": review,
        "qualification": {"anatomical_surface_geometry": "unverified", "physical_evidence": "unmeasured",
                          "mechanical_transforms_validated": False, "fabrication_released": False},
        "open_evidence": [
            "Whole-bone envelopes remain unknown for 24 unfused vertebrae and coccyx.",
            "Sacrum dataset spans and estimated mass are unverified legacy inputs, not reviewed osteometry.",
            "C2HT, T6_VBWs and T11_VBHp remain quarantined; no substitute is inferred.",
            "Pedicle/lamina/process sizes, articular surfaces, facet positions, discs and soft-tissue interfaces remain unresolved.",
            "Studio spine ellipsoids are provisional display proxies; none models the reviewed features.",
            "Anatomical superior references do not qualify mechanical joints, limits, signal routing or load paths.",
        ],
        "inputs": {"spine_declarations_sha256": sha256(json_bytes(declarations)),
                   "features_sha256": sha256((root / FEATURES_PATH).read_bytes()),
                   "source_review_sha256": sha256((root / SOURCE_REVIEW_PATH).read_bytes()),
                   "dimension_catalog_sha256": sha256((root / "references/dim072/catalog.json").read_bytes()),
                   "measurement_protocols": dimension_catalog["measurement_protocols"],
                   "native_snapshots": {key: dimension_catalog["sources"][key]["native_snapshot"] for key in sorted(reviewed_sources)},
                   "datasets": sorted({sha256(json_bytes(bone.dataset)) for bone in bones if bone.unique_id in SPINE_ORDER and bone.dataset is not None})},
    }


def markdown_report(report):
    lines = ["# PHY spine review", "",
             f"Records: **{report['counts']['present_spine_bones']}/26**. Admitted feature spans checked: **{report['counts']['admitted_feature_fields_checked']}**.",
             f"Data consistency: **{report['consistency_pass']}**. All dimensions verified: **false**. Fabrication release: **false**.",
             "", "Published-table transcription and qualitative anatomy are reviewed separately from geometry and physical evidence.",
             "The source review checked 105 published rows, including three quarantined rows, without changing a numerical value.",
             "", "| Bone | Dimensional status | Admitted spans | Whole-bone unknowns | Superior reference |",
             "| --- | --- | ---: | --- | --- |"]
    for row in report["bones"]:
        if row["record_status"] == "missing_record":
            lines.append(f"| {row['bone_id']} | missing record | - | unknown | unresolved |")
        else:
            lines.append(f"| {row['bone_id']} | {row['dimensional_status']} | {len(row['admitted_feature_fields'])} | {', '.join(row['whole_bone_unknown_fields']) or 'legacy values only'} | {row['anatomical_parent']['bone_id'] or 'unresolved'} |")
    lines += ["", "## Declared numerical values", "",
              "All values below are unscaled millimeters. Named body spans and osseous opening spans are not whole-bone envelopes or usable mechanical clearances.",
              "Unknown whole-bone fields remain explicit in the inventory table and JSON. SD is cohort variation, not a manufacturing tolerance.",
              "", "| Bone | Field | Value / mm | Basis |", "| --- | --- | ---: | --- |"]
    for row in report["bones"]:
        if row["record_status"] == "present":
            for key, value in sorted(row["dimensions_mm"].items()):
                if value is not None:
                    basis = "admitted feature span" if key in row["admitted_feature_fields"] else "legacy dataset / unverified"
                    lines.append(f"| {row['bone_id']} | {key} | {value:g} | {basis} |")
    lines += ["", "## Qualitative feature declarations", ""]
    for row in report["bones"]:
        if row["record_status"] == "present":
            lines += [f"- **{row['bone_id']}**: " + " ".join(row["notable_features"])]
    lines += ["", "## Consistency issues", ""]
    lines += [f"- {issue['bone_id']} / {issue['check']}: {issue['detail']}" for issue in report["issues"]] or ["- None in the reviewed data contract."]
    lines += ["", "## Open evidence", ""] + ["- " + item for item in report["open_evidence"]]
    lines += ["", "## Qualitative references", ""]
    lines += [f"- [{source['title']}]({source['url']}): {source['scope']}" for source in report["sources"].values()]
    lines += ["", "## Published numerical review", ""]
    for source in report["published_source_review"]["numerical_reviews"]:
        url = source["source_response"]["url"]
        lines.append(f"- [{source['source_id']}]({url}): {source['rows_checked']} printed rows checked against {source['table']}; {source['admitted_rows']} admitted. {source['method']}.")
    return "\n".join(lines) + "\n"


def write_reports(report, output_root):
    destination = Path(output_root) / "reports"
    files = {"spine_audit.json": json_bytes(report), "spine_audit.md": markdown_report(report).encode("utf-8")}
    manifest = {"audit_version": AUDIT_VERSION, "inputs": report["inputs"], "fabrication_released": False,
                "files": {name: {"sha256": sha256(content), "size_bytes": len(content)} for name, content in files.items()}}
    files["spine_audit_manifest.json"] = json_bytes(manifest)
    publish_files({destination / name: content for name, content in files.items()}, destination / "spine_audit_manifest.json")
    return {name: str(destination / name) for name in files}
