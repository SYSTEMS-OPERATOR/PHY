"""Read-only, typed relationship snapshots; no geometry or routing synthesis.

This module uses only the standard library and PHY's identity/catalog contracts.
BoneSpec remains authoritative. A source-file correspondence does not establish
that a caller-supplied object was constructed from those file bytes.
"""
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

from skeleton.anatomical_inventory import ROOT, INVENTORY_VERSION, expected_bones, source_record_paths
from skeleton.identity import IdentityResolver, find_parent_cycles
from skeleton.relationship_catalog import CATALOG_PATH, load_catalog, validate_catalog

RELATIONSHIP_VERSION = "1.0.0"
SCHEMA_PATH = "skeleton/schema/relationship_graph.schema.json"
RELATIONSHIP_KINDS = (
    "legacy_parent", "legacy_adjacency", "legacy_articulation",
    "qualitative_articulation", "structural_membership", "operational_signal",
)
RESOLVED = {"id", "unique_display_name"}


def json_safe(value, seen=None):
    """Retain JSON values; tag invalid values without unstable object reprs."""
    if value is None or type(value) in (str, bool):
        return value
    if type(value) in (int, float):
        try:
            if math.isfinite(value):
                return value
        except OverflowError:
            return {"invalid_type": "integer", "hex": hex(value)}
        return {"invalid_type": "float", "value": str(value)}
    seen = set() if seen is None else seen
    if id(value) in seen:
        return {"invalid_type": "cyclic_reference"}
    if isinstance(value, (list, dict)):
        seen = seen | {id(value)}
        if isinstance(value, list):
            return [json_safe(item, seen) for item in value]
        if all(isinstance(key, str) for key in value):
            return {key: json_safe(item, seen) for key, item in value.items()}
        entries = [[json_safe(key, seen), json_safe(item, seen)] for key, item in value.items()]
        return {"invalid_type": "mapping_keys", "entries": sorted(entries, key=encode)}
    return {"invalid_type": type(value).__module__ + "." + type(value).__qualname__}


def encode(value):
    return (json.dumps(value, sort_keys=True, indent=2, allow_nan=False) + "\n").encode("utf-8")


def sha256(value):
    return hashlib.sha256(value).hexdigest()


def _status(participants):
    statuses = {p["resolution"]["status"] for p in participants}
    for status in ("invalid", "ambiguous", "missing", "unspecified"):
        if status in statuses:
            return status
    return "resolved"


def build_relationship_snapshot(field, *, root=ROOT, catalog=None):
    """Observe supplied bones and current runtime links without changing either.

    Adult inventory slots always total 206. Missing records stay missing;
    non-inventory inputs are listed separately, never added to adult coverage.
    Catalog participants resolve only against the supplied snapshot. Runtime
    links use domain IDs, whereas anatomical declarations use canonical IDs.
    """
    root = Path(root)
    supplied = list(field.bones.values())
    resolver = IdentityResolver({"unique_id": b.unique_id, "name": b.name} for b in supplied)
    if resolver.fatal:
        raise ValueError("Invalid relationship identities: " + json.dumps({
            "invalid_ids": resolver.invalid_ids, "duplicate_ids": resolver.duplicate_ids}, sort_keys=True))
    supplied.sort(key=lambda bone: bone.unique_id)
    by_id = {b.unique_id: b for b in supplied}
    registrations = {b.unique_id: key for key, b in field.bones.items()}
    runtime = IdentityResolver({"unique_id": key, "name": b.name} for key, b in field.bones.items())
    domains = {key: b.unique_id for key, b in field.bones.items() if isinstance(key, str)}
    inventory = expected_bones()
    expected = {row["bone_id"] for row in inventory}
    repository_catalog = catalog is None
    catalog = json_safe(load_catalog(root) if repository_catalog else catalog)
    validate_catalog(catalog)
    catalog_hash = sha256(encode(catalog))
    catalog_file_hash = sha256((root / CATALOG_PATH).read_bytes()) if repository_catalog else None
    paths = source_record_paths(root)
    sources = {uid: {"path": path, "sha256": sha256((root / path).read_bytes())}
               for uid, path in sorted(paths.items()) if uid in by_id}
    raw = [{"bone_id": b.unique_id, "name": json_safe(b.name), "domain_id": json_safe(b.domain_id),
            "registered_domain_id": json_safe(registrations[b.unique_id]),
            **{key: json_safe(getattr(b, key)) for key in (
                "connections", "articulations", "entanglement_links", "source_ids", "references")}}
           for b in supplied]
    relations, parents = [], {}

    def participant(reference, role, *, policy="exact", optional=False, compatibility=False):
        resolution = resolver.resolve(reference, policy=policy, optional=optional)
        if resolution["status"] == "invalid":
            resolution["reference"] = json_safe(reference)
        value = {"role": role, "reference_value": json_safe(reference),
                 "bone_id": resolution["bone_id"], "matching_policy": policy,
                 "resolution": json_safe(resolution)}
        if compatibility:
            runtime_result = resolver.resolve(reference, policy="case_insensitive")
            if runtime_result["status"] == "invalid":
                runtime_result["reference"] = json_safe(reference)
            value["runtime_resolution"] = json_safe(runtime_result)
        return value

    def add(rid, kind, role, original, participants, provenance, **details):
        relations.append({
            "relationship_id": rid, "kind": kind, "semantic_role": role,
            "participants": participants, "original_declaration": json_safe(original),
            "resolution_status": _status(participants), "provenance": provenance,
            "biological_classification": None, "qualitative_motion": [],
            "engineering_definition": None, **details,
        })

    for bone in supplied:
        uid = bone.unique_id
        owner = participant(uid, "declaring_bone")

        def provenance(field_name, value):
            return {"kind": "BoneSpec_snapshot", "bone_id": uid, "field": field_name,
                    "source_path": sources.get(uid, {}).get("path"),
                    "source_file_sha256": sources.get(uid, {}).get("sha256"),
                    "source_correspondence_only": True,
                    "value_sha256": sha256(encode(json_safe(value))),
                    "source_ids": json_safe(bone.source_ids), "references": json_safe(bone.references)}

        def declaration(kind, path, original, target, role, *, optional=False, compatibility=False, invalid=False):
            participants = [owner, participant(target, "declared_target", optional=optional,
                                                compatibility=compatibility)]
            if invalid:
                participants[1].update(bone_id=None)
                participants[1]["resolution"].update(bone_id=None, status="invalid")
                if "runtime_resolution" in participants[1]:
                    participants[1]["runtime_resolution"].update(bone_id=None, status="invalid")
            add(f"REL:{uid}:{path}", kind, role, original, participants, provenance(path, original))
            return participants[1]["bone_id"]

        connections = bone.connections
        if isinstance(connections, dict):
            parent = connections.get("parent")
            parents[uid] = declaration("legacy_parent", "connections.parent", parent, parent,
                                       "declared_anatomical_parent_reference", optional=True)
            children = connections.get("children", [])
        else:
            parents[uid] = None
            declaration("legacy_parent", "connections", connections, connections,
                        "invalid_parent_container", invalid=True)
            children = []
        if not isinstance(children, list):
            declaration("legacy_adjacency", "connections.children", children, children,
                        "invalid_adjacency_container", invalid=True)
        else:
            for index, child in enumerate(children):
                declaration("legacy_adjacency", f"connections.children[{index}]", child, child,
                            "declared_anatomical_adjacency")
        articulations = bone.articulations
        if not isinstance(articulations, list):
            declaration("legacy_articulation", "articulations", articulations, articulations,
                        "invalid_articulation_container", invalid=True)
        else:
            for index, art in enumerate(articulations):
                target = art.get("bone") if isinstance(art, dict) else art
                declaration("legacy_articulation", f"articulations[{index}]", art, target,
                            "legacy_anatomical_articulation", compatibility=True, invalid=not isinstance(art, dict))
        links = bone.entanglement_links
        invalid_links = not isinstance(links, list)
        if invalid_links:
            links = [links]
        for index, link in enumerate(links):
            result = runtime.resolve(link, policy="id_only")
            if invalid_links:
                result.update(bone_id=None, status="invalid")
            if result["status"] == "invalid":
                result["reference"] = json_safe(link)
            domain = result["bone_id"]
            result = {**result, "runtime_domain_id": domain,
                      "bone_id": domains.get(domain) if domain is not None else None}
            target = {"role": "runtime_peer", "reference_value": json_safe(link),
                      "bone_id": result["bone_id"], "matching_policy": "runtime_domain_id_only",
                      "resolution": json_safe(result)}
            path = "entanglement_links" if invalid_links else f"entanglement_links[{index}]"
            add(f"REL:{uid}:{path}", "operational_signal", "observed_runtime_signal_link", link,
                [owner, target], provenance(path, link), direction="declaring_bone_to_runtime_peer")

    for group, key, kind, role in (("regions", "region_id", "structural_membership", "regional_participation"),
                                    ("articulations", "articulation_id", "qualitative_articulation", "qualitative_anatomical_articulation")):
        for row in sorted(catalog[group], key=lambda item: item[key]):
            locator = f"{group}/{row[key]}"
            provenance = {"kind": "curated_catalog", "source_path": CATALOG_PATH if repository_catalog else None,
                          "catalog_locator": locator, "reference_locator": row["locator"],
                          "catalog_sha256": catalog_hash, "source_file_sha256": catalog_file_hash,
                          "value_sha256": sha256(encode(row)), "source_ids": [row["source_id"]]}
            if group == "regions":
                participants = [participant(c["bone_id"], c["role"]) for c in row["contributions"]]
                details = {"region_id": row[key], "name": row["name"], "spatial_definition": row["spatial_definition"]}
            else:
                participants = [participant(uid, "articular_bone") for uid in row["bone_ids"]]
                details = {"articulation_id": row[key], "name": row["name"],
                           "biological_classification": row["anatomical_type"],
                           "qualitative_motion": list(row["motion_components"]),
                           "non_bone_components": list(row["non_bone_components"]),
                           "engineering_definition": row["engineering_definition"]}
            add("REL:catalog:" + locator, kind, role, row, participants, provenance, **details)

    relations.sort(key=lambda row: row["relationship_id"])
    unresolved = []
    for relation in relations:
        for index, p in enumerate(relation["participants"]):
            for projection, r in (("declared", p["resolution"]), ("runtime_compatibility", p.get("runtime_resolution"))):
                if r is not None and r["status"] not in RESOLVED | {"unspecified"}:
                    unresolved.append({"relationship_id": relation["relationship_id"], "participant": index,
                                       "projection": projection, **r})
    graph = {
        "relationship_version": RELATIONSHIP_VERSION, "inventory_version": INVENTORY_VERSION,
        "scope": "declared_anatomy_curated_participation_and_observed_runtime",
        "inventory": {"expected_adult_identities": len(inventory),
                      "loaded_adult_identities": len(expected & resolver.ids),
                      "supplied_record_count": len(supplied),
                      "missing_record_ids": sorted(expected - resolver.ids),
                      "non_inventory_record_ids": sorted(resolver.ids - expected),
                      "complete_adult_snapshot": expected == resolver.ids},
        "bones": [{**row, "record_status": "present" if row["bone_id"] in by_id else "missing_record"}
                  for row in inventory],
        "supplemental_records": [{"bone_id": uid, "name": json_safe(by_id[uid].name),
                                  "adult_inventory_member": False} for uid in sorted(resolver.ids - expected)],
        "regions": [{"region_id": row["region_id"], "name": row["name"], "entity_kind": "region"}
                    for row in sorted(catalog["regions"], key=lambda row: row["region_id"])],
        "sources": catalog["sources"], "relationships": relations,
        "counts_by_kind": {kind: Counter(r["kind"] for r in relations)[kind] for kind in RELATIONSHIP_KINDS},
        "diagnostics": {"unresolved_references": unresolved, "parent_cycles": find_parent_cycles(parents),
                        "invalid_runtime_ids": runtime.invalid_ids, "duplicate_runtime_ids": runtime.duplicate_ids,
                        "runtime_binding_issues": [{"bone_id": b.unique_id,
                            "registered_domain_id": json_safe(registrations[b.unique_id]),
                            "declared_domain_id": json_safe(b.domain_id)} for b in supplied
                            if registrations[b.unique_id] != b.domain_id]},
        "matching_policies": {"runtime_legacy_names": "case_insensitive", "rendering": "exact",
                              "export_parent_and_adjacency": "exact", "observed_signal_links": "runtime_domain_id_only"},
        "inputs": {"inventory_sha256": sha256(encode(inventory)),
                   "bone_declarations_sha256": sha256(encode(raw)),
                   "source_files": sources, "catalog_path": CATALOG_PATH if repository_catalog else None,
                   "catalog_sha256": catalog_hash, "catalog_file_sha256": catalog_file_hash,
                   "schema": {"path": SCHEMA_PATH, "sha256": sha256((root / SCHEMA_PATH).read_bytes())}},
        "qualification": {"canon_effect": "none", "mechanical_transforms_validated": False,
                          "engineering_definitions_qualified": False, "fabrication_released": False},
    }
    validate_relationship_snapshot(graph)
    return graph


def validate_relationship_snapshot(graph):
    """Enforce publication boundaries without a third-party schema dependency.

    The JSON Schema supplies the complete interchange shape; these independent
    checks also enforce inventory identity, unique declarations and value hashes.
    """
    if graph.get("relationship_version") != RELATIONSHIP_VERSION or graph.get("inventory_version") != INVENTORY_VERSION:
        raise ValueError("Unsupported relationship contract version")
    if graph.get("scope") != "declared_anatomy_curated_participation_and_observed_runtime":
        raise ValueError("Unsupported relationship scope")
    if graph.get("qualification") != {"canon_effect": "none", "mechanical_transforms_validated": False,
                                      "engineering_definitions_qualified": False, "fabrication_released": False}:
        raise ValueError("Relationships cannot qualify engineering or fabrication")
    expected = {r["bone_id"] for r in expected_bones()}
    bones = graph["bones"]
    if len(bones) != 206 or {b["bone_id"] for b in bones} != expected:
        raise ValueError("Relationship adult inventory must contain exactly the 206 identity slots")
    present = {b["bone_id"] for b in bones if b["record_status"] == "present"}
    extra = {b["bone_id"] for b in graph["supplemental_records"]}
    inventory = graph["inventory"]
    if (inventory["expected_adult_identities"] != 206 or inventory["loaded_adult_identities"] != len(present)
            or set(inventory["missing_record_ids"]) != expected - present
            or set(inventory["non_inventory_record_ids"]) != extra or extra & expected
            or inventory["supplied_record_count"] != len(present) + len(extra)
            or inventory["complete_adult_snapshot"] != (present == expected and not extra)):
        raise ValueError("Inconsistent relationship inventory coverage")
    seen, counts = set(), Counter()
    for row in graph["relationships"]:
        rid = row["relationship_id"]
        if rid in seen or row["kind"] not in RELATIONSHIP_KINDS:
            raise ValueError("Invalid or duplicate relationship identity/kind")
        seen.add(rid)
        counts[row["kind"]] += 1
        if row["engineering_definition"] is not None:
            raise ValueError("Engineering definitions remain null in relationship contract 1.0.0")
        if row["provenance"]["value_sha256"] != sha256(encode(row["original_declaration"])):
            raise ValueError("Relationship original value differs from its provenance hash")
        if any(p["bone_id"] is not None and p["bone_id"] not in present | extra for p in row["participants"]):
            raise ValueError("A relationship cannot resolve to a missing record")
    if graph["counts_by_kind"] != {kind: counts[kind] for kind in RELATIONSHIP_KINDS}:
        raise ValueError("Relationship counts differ from declarations")
    encode(graph)


def relationships_for_bone(graph, bone_id):
    """Include unresolved declarations naming this ID as an ambiguity candidate."""
    return [row for row in graph["relationships"] if any(
        p["bone_id"] == bone_id or bone_id in p["resolution"].get("candidates", [])
        or bone_id in p.get("runtime_resolution", {}).get("candidates", []) for p in row["participants"])]


def relationship_markdown(graph):
    def cell(value):
        return str(value).replace("|", "\\|").replace("\n", " ").replace("\r", " ")
    inventory = graph["inventory"]
    lines = ["# PHY relationship inspection / " + graph["relationship_version"], "",
             f"Adult records present: {inventory['loaded_adult_identities']} / {inventory['expected_adult_identities']}.",
             "Missing records, regions and supplemental records do not increase adult coverage.",
             "Declarations, qualitative anatomy, regional membership and runtime links are separate. No mechanical definition or fabrication release is granted.", "",
             "| Kind | Declarations |", "| --- | ---: |"]
    lines += [f"| {kind} | {count} |" for kind, count in graph["counts_by_kind"].items()]
    lines += ["", "## Selected curated relationships", "", "| ID | Role | Participants | Engineering definition |",
              "| --- | --- | --- | --- |"]
    for row in graph["relationships"]:
        if row["kind"] in ("structural_membership", "qualitative_articulation"):
            participants = ", ".join(p["bone_id"] or "unresolved:" + str(p["reference_value"]) for p in row["participants"])
            lines.append(f"| {cell(row['relationship_id'])} | {row['kind']} | {cell(participants)} | null |")
    lines += ["", "## Unresolved declarations and compatibility projections", "",
              "Exact export/render matching and case-insensitive legacy runtime matching remain distinct.", "",
              "| Declaration | Projection | Reference | Status | Candidates |", "| --- | --- | --- | --- | --- |"]
    unresolved = graph["diagnostics"]["unresolved_references"]
    # Keep the initial TMJ review visible; complete diagnostics stay in JSON.
    ordered = sorted(unresolved, key=lambda row: not row["relationship_id"].startswith("REL:BONE_MANDIBLE:"))
    for row in ordered[:30]:
        lines.append("| " + " | ".join(cell(row.get(key, "")) for key in
                     ("relationship_id", "projection", "reference", "status", "candidates")) + " |")
    lines += ["", f"Showing {min(30, len(unresolved))} of {len(unresolved)} unresolved resolution diagnostics; all remain in JSON.",
              "", "Parent cycles (declared parent projection only): `" + json.dumps(graph["diagnostics"]["parent_cycles"]) + "`.",
              "", "Complete original values, source locators, content hashes and null definitions are retained in relationship_graph.json.",
              "Source-file correspondence alone does not prove that a supplied snapshot was loaded from that file.", ""]
    return "\n".join(lines)
