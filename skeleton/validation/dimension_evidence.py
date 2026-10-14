"""Bounded offline review of PHY-DIM-072 tabular derivations.

Checks custody, numerical derivation, cohort/side declarations and nonregression.
It does not authenticate a publication, qualify anatomy or release fabrication.
"""
from __future__ import annotations

from copy import deepcopy
import hashlib
import json
from pathlib import Path

from skeleton.anatomical_inventory import expected_bones
from skeleton.exporters.exporter_agent import json_bytes
from skeleton.exporters.identity import ExportIdentity
from skeleton.validation.validator_agent import finite_number

ROOT = Path(__file__).resolve().parents[2]
CATALOG_PATH = Path("references/dim072/catalog.json")
TOOL_VERSION = "dim072-tabular-v1"


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"Duplicate JSON key: {key}")
        result[key] = value
    return result


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"),
                      object_pairs_hook=_unique_object,
                      parse_constant=lambda value: (_ for _ in ()).throw(
                          ValueError(f"Nonfinite JSON token: {value}")))


def _text(value, label):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"Missing provenance: {label}")


def _digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _pinned_file(root, descriptor):
    path = descriptor.get("path")
    if not isinstance(path, str) or Path(path).is_absolute():
        raise ValueError("Evidence path must be repository-relative")
    root = Path(root).resolve()
    candidate = (root / path).resolve()
    if not candidate.is_relative_to(root):
        raise ValueError("Evidence path escapes repository")
    digest = descriptor.get("sha256")
    if not isinstance(digest, str) or len(digest) != 64 or _digest(candidate) != digest:
        raise ValueError(f"Evidence hash mismatch: {path}")
    return candidate


def provenance_ref(index, field=None):
    ref = f"{CATALOG_PATH.as_posix()}#/candidates/{index}"
    return ref if field is None else f"{ref}/fields/{field}"


def derive_value(measurement, mapping, bone_id, side):
    """Identity transform of a reviewed published millimeter statistic.

    No mesh axes, stature scaling, inferred symmetry or spatial registration are
    supported by this narrowly scoped tool. A new recipe needs a separate review.
    """
    if measurement.get("admissible") is not True:
        raise ValueError(f"Rejected measurement for {bone_id}")
    if measurement.get("native_units") != "mm":
        raise ValueError("Native units must be explicitly mm for this recipe")
    if mapping.get("method") != "published_mean_identity":
        raise ValueError("Unsupported derivation method")
    if "canonical_registration" not in mapping:
        raise ValueError("Missing provenance: explicit coordinate registration status")
    scale = mapping.get("unit_scale")
    if not finite_number(scale) or scale != 1.0:
        raise ValueError("Unreviewed scale or coordinate transform")
    if mapping.get("canonical_registration") is not None:
        raise ValueError("Unreviewed canonical coordinate registration")
    for key in ("mean", "sd"):
        value = measurement.get(key)
        if not finite_number(value) or (value <= 0 if key == "mean" else value < 0):
            raise ValueError(f"Invalid native {key} for {bone_id}")
    _text(mapping.get("meaning"), "dimensional meaning")
    native_id = measurement.get("bone_id")
    native_side = measurement.get("side")
    if native_side == "pooled":
        if (mapping.get("side_application") != "pooled_mean_default"
                or native_id != bone_id.rsplit("_", 1)[0]
                or side not in ("left", "right")):
            raise ValueError("Pooled statistic requires an explicit bilateral default recipe")
        _text(mapping.get("side_assumption"), "pooled side assumption")
    elif native_id != bone_id or native_side != side:
        raise ValueError("Native bone identity or left/right correspondence mismatch")
    return float(measurement["mean"])


def load_evidence(root=ROOT, catalog=None):
    """Resolve frozen fact snapshots; an intact hash is not source admission."""
    root = Path(root)
    catalog = read_json(root / CATALOG_PATH) if catalog is None else deepcopy(catalog)
    if (catalog.get("schema_version") != "PHY_DIM072_TABULAR_V1"
            or catalog.get("tool_version") != TOOL_VERSION
            or catalog.get("assignment") != "PHY-DIM-072"
            or catalog.get("target") != 72):
        raise ValueError("Unknown assignment or evidence contract")
    protocols = read_json(_pinned_file(root, catalog.get("measurement_protocols", {})))
    if protocols.get("schema_version") != "PHY_DIM072_PROTOCOL_V1":
        raise ValueError("Unknown anatomical measurement protocol")
    baseline = read_json(_pinned_file(root, catalog["baseline_snapshot"]))
    if (catalog.get("phy_commit") != baseline.get("phy_commit")
            or catalog.get("phy_ref_commit") != baseline.get("phy_ref_commit")):
        raise ValueError("Baseline commit/provenance mismatch")
    if catalog.get("canonical_reference") != {
            "version": "1.0.0", "height_mm": 1676.4, "span_mm": 1676.4, "registered": False}:
        raise ValueError("Unreviewed canon or embodiment registration")
    inventory = {row["bone_id"]: row for row in expected_bones()}
    rows = baseline.get("bones", [])
    if (len(rows) != 206 or len({row["bone_id"] for row in rows}) != 206
            or {row["bone_id"] for row in rows} != set(inventory)):
        raise ValueError("Baseline must retain exactly the 206 adult identities")
    prior = {row["bone_id"]: row for row in rows}
    source_rows = {}
    for key, source in catalog["sources"].items():
        if source.get("kind") != "primary_bone_morphometry":
            raise ValueError(f"Source scope cannot supply bone dimensions: {key}")
        for name in ("title", "doi", "revision", "cohort", "locator", "evidence_review",
                     "rights_review", "publisher_license", "coordinate_frame", "extraction",
                     "tool_version", "uncertainty_policy", "retrieved_on"):
            _text(source.get(name), f"{key}.{name}")
        if (source.get("native_units") != "mm"
                or source.get("geometry_qualified") is not False
                or source.get("fabrication_qualified") is not False):
            raise ValueError("Tabular admission does not qualify geometry or fabrication")
        facts = read_json(_pinned_file(root, source["native_snapshot"]))
        if facts.get("source_id") != key:
            raise ValueError("Fact snapshot/source mismatch")
        _text(facts.get("table"), "table locator")
        parameters = [row["parameter"] for row in facts["rows"]]
        if len(parameters) != len(set(parameters)):
            raise ValueError("Duplicate native measurement parameter")
        source_rows[key] = {row["parameter"]: row for row in facts["rows"]}
    candidates = catalog["candidates"]
    ids = [row["bone_id"] for row in candidates]
    if len(ids) != 36 or len(set(ids)) != 36 or not set(ids) <= set(inventory):
        raise ValueError("Need 36 distinct existing candidate identities")
    if [row["rank"] for row in candidates] != list(range(1, 37)):
        raise ValueError("Candidate ranks must be unique and ordered")
    derived = {}
    for candidate in candidates:
        uid = candidate["bone_id"]
        if (prior[uid]["dimensions"]["status"] == "complete"
                or candidate["baseline_missing_fields"] != prior[uid]["dimensions"]["unknown_keys"]):
            raise ValueError("Candidate must be incomplete in the pinned baseline")
        if candidate["side"] != inventory[uid]["side"] or candidate["region"] != inventory[uid]["region"]:
            raise ValueError("Candidate inventory correspondence mismatch")
        if candidate.get("geometry_qualified") is not False or candidate.get("fabrication_qualified") is not False:
            raise ValueError("No physical qualification may be inferred")
        values = {}
        for field, mapping in candidate["fields"].items():
            if not field.endswith("_mm"):
                raise ValueError("Resulting field must explicitly use mm")
            if mapping["source_id"] not in source_rows:
                raise ValueError("Missing source provenance")
            try:
                measure = source_rows[mapping["source_id"]][mapping["parameter"]]
            except KeyError as error:
                raise ValueError("Missing native measurement provenance") from error
            if measure.get("admitted_field") != field:
                raise ValueError("Dimensional semantics differ from the reviewed admission")
            try:
                protocol_source = protocols["sources"][mapping["source_id"]]
                protocol = protocol_source["fields"][field]
            except KeyError as error:
                raise ValueError("Missing anatomical landmark protocol") from error
            _text(protocol.get("landmark_definition"), "anatomical landmark definition")
            _text(protocol_source.get("source_locator"), "landmark method locator")
            _text(protocol_source.get("uncertainty"), "measurement uncertainty")
            if (protocol.get("native_units") != "mm"
                    or protocol_source.get("geometry_qualified") is not False
                    or protocol_source.get("fabrication_qualified") is not False):
                raise ValueError("Landmark method cannot qualify geometry or fabrication")
            values[field] = derive_value(measure, mapping, uid, candidate["side"])
        derived[uid] = values
    return catalog, baseline, derived


def expansion_report(field, engineering, root=ROOT, catalog=None):
    """Compare all 206 maps, retaining every prior unknown unless supported."""
    catalog, baseline, derived = load_evidence(root, catalog)
    bones = {}
    for bone in field.bones.values():
        if bone.unique_id in bones:
            raise ValueError("Duplicate current bone identity")
        bones[bone.unique_id] = bone
    prior = {row["bone_id"]: row for row in baseline["bones"]}
    if set(bones) != set(prior):
        raise ValueError("Current inventory must retain exactly 206 identities")
    records = [bones[uid].to_fabrication_record() for uid in sorted(bones)]
    identity = ExportIdentity(records)
    if identity.fatal or identity.report["parent_cycles"]:
        raise ValueError("Duplicate identity or parent cycle")
    current = {row["unique_id"]: row for row in records}
    rows = []; complete = []; additions = []; partial = []
    indices = {row["bone_id"]: i for i, row in enumerate(catalog["candidates"])}
    for uid, old in sorted(prior.items()):
        bone = bones[uid]; before = old["dimensions"]["values_mm"]
        after = current[uid]["dimensions"]
        expected = dict(before); expected.update(derived.get(uid, {}))
        if after != expected:
            raise ValueError(f"Unsupported dimension change, dropped unknown or regression: {uid}")
        if any(v is not None and (not finite_number(v) or v <= 0) for v in after.values()):
            raise ValueError(f"Invalid canonical dimension: {uid}")
        new = derived.get(uid, {})
        if new:
            index = indices[uid]
            if provenance_ref(index) not in bone.references:
                raise ValueError(f"Absent bone provenance: {uid}")
            for key in new:
                if bone.metric_sources.get(key) != provenance_ref(index, key):
                    raise ValueError(f"Absent value provenance: {uid}.{key}")
            if (current[uid]["physics"].get("mass_kg") is not None
                    or bone.joint_interfaces or bone.mount_points):
                raise ValueError("Tabular evidence cannot resolve mass or physical interfaces")
        is_complete = bool(after) and all(v is not None for v in after.values())
        if is_complete:
            complete.append(uid)
            if old["dimensions"]["status"] != "complete": additions.append(uid)
        elif new: partial.append(uid)
        rows.append({"bone_id": uid, "region": old["region"], "side": old["side"],
            "before_mm": before, "after_mm": after,
            "declared_complete": is_complete, "source_backed_complete": uid in additions,
            "new_source_backed_fields": sorted(new),
            "geometry_qualified": False, "fabrication_qualified": False})
    observed = {row["bone_id"]: row["dimensions"]["values_mm"] for row in engineering["bones"]}
    if observed != {uid: row["dimensions"] for uid, row in current.items()}:
        raise ValueError("Engineering audit was not generated from the current snapshot")
    if engineering["metrics"]["declared_dimensions_complete"]["count"] != len(complete):
        raise ValueError("Engineering and expansion counts disagree")
    return {
        "assignment": "PHY-DIM-072", "tool_version": TOOL_VERSION,
        "baseline_commits": {"PHY": catalog["phy_commit"], "PHY-REF": catalog["phy_ref_commit"]},
        "inventory_count": len(rows), "target_complete": 72,
        "baseline_complete": sum(b["dimensions"]["status"] == "complete" for b in prior.values()),
        "current_complete": len(complete), "new_complete_ids": additions,
        "improved_partial_ids": partial, "new_source_backed_field_count": sum(map(len, derived.values())),
        "remaining_to_target": max(0, 72-len(complete)), "target_met": len(complete) >= 72,
        "evidence_integrity_pass": True,
        "record_validation_pass": engineering["metrics"]["record_validation_pass"]["count"],
        "geometry_qualified_additions": 0, "fabrication_qualified_additions": 0,
        "fabrication_released": False, "rows": rows,
        "source_facts": {key: value["native_snapshot"] for key, value in catalog["sources"].items()},
        "inputs_sha256": {
            "catalog": _digest(Path(root)/CATALOG_PATH),
            "baseline": catalog["baseline_snapshot"]["sha256"],
            "engineering_snapshot": hashlib.sha256(json_bytes(engineering)).hexdigest()},
    }


def markdown_report(report, catalog):
    lines = ["# PHY-DIM-072 evidence and dimension expansion", "",
        f"**Result: {report['current_complete']}/206 complete declared maps; target 72 is {'met' if report['target_met'] else 'not met'}.**", "",
        f"The pinned baseline has {report['baseline_complete']} complete maps. Added {len(report['new_complete_ids'])} complete maps and improved {len(report['improved_partial_ids'])} partial maps with {report['new_source_backed_field_count']} traceable fields. Remaining shortfall: {report['remaining_to_target']} maps.", "",
        "Coverage counts finite declarations. Source-backed fields retain study scope, cohort and limitations. No numerical data is supplied by Studio proxies, external anthropometric subtraction, uniform stature scaling or rejected references.", "",
        f"Current full record validation passes: {report['record_validation_pass']}/206. No geometry/fabrication qualification or physical release is claimed.", "",
        "## Ranked source-to-dimension matrix", "",
        "| Rank | Identity | Region | New fields | Complete | Remaining declared unknowns | Source |",
        "| ---: | --- | --- | ---: | --- | --- | --- |"]
    rows = {row["bone_id"]: row for row in report["rows"]}
    for c in catalog["candidates"]:
        row = rows[c["bone_id"]]
        refs = sorted({m["source_id"] for m in c["fields"].values()})
        unknown = ", ".join(k for k,v in row["after_mm"].items() if v is None)
        lines.append(f"| {c['rank']} | {c['bone_id']} | {c['region']} | {len(c['fields'])} | {'yes' if row['declared_complete'] else 'no'} | {unknown or 'none'} | {', '.join(refs) or 'unresolved'} |")
    lines += ["", "## Changed values and derivations", "",
        "Native factual extracts remain under references/dim072. Every applied value uses the published mean unchanged in mm. Published SD measures cohort variation; it is not a tolerance. No whole-body registration is performed.", "",
        "| Identity | Canonical field | Before mm | After mm | Source / native row |", "| --- | --- | --- | ---: | --- |"]
    for c in catalog["candidates"]:
        row = rows[c["bone_id"]]
        for key,m in sorted(c["fields"].items()):
            value = row["after_mm"][key]
            lines.append(f"| {c['bone_id']} | {key} | {row['before_mm'].get(key)} | {value:g} | {m['source_id']} / {m['parameter']} |")
    lines += ["", "## Source scope and limits", ""]
    for key,source in sorted(catalog["sources"].items()):
        lines += [f"- **{key}:** [{source['title']}]({source['locator']}). Revision: {source['revision']}. {source['cohort']}",
                  f"  {source['extraction']} {' '.join(source['limitations'])}"]
    lines += ["", "## Unresolved evidence", "",
        "The 24 spine identities retain their original whole-bone length/width/thickness unknowns. Body/endplate heights and canal void spans are independently named. C1 has no body height, C2 HT has unresolved odontoid semantics, and the inconsistent T11 posterior-height entry is quarantined.", "",
        "Calcaneal body/wall spans do not resolve whole-bone width/thickness. First-rib tubercle spans exclude neck/head, so they do not become total rib length. Ribs 2 and the naviculars have no admitted three-span defaults. Existing sacrum, sternum, hips and long-bone maps remain legacy declarations; they are not promoted by this work.", "",
        "The source review, NASA crosswalk and NIH candidate disposition are in docs/PHY-DIM-072.md. No unqualified mesh measurements are used.", "",
        "## Next physical work", "",
        "1. Resolve the L5/sacrum/hip datums with a metric-fidelity-reviewed segmented female pelvis reference; establish the whole-vertebra span contract.",
        "2. Define a bounded adjacent lumbar cartridge: neutral/end-stop datums, load path, bearing/stop basis, mounting tolerances and inspection landmarks.",
        "3. Define tibia/talus/calcaneus contact and ankle datums; obtain unscaled geometry and full calcaneal/navicular dimensions before foot support fabrication.",
        "4. Resolve the shoulder/thorax root and existing A0 bearing/stop blockers independently; do not infer a release from anatomical coverage.", ""]
    return "\n".join(lines)
