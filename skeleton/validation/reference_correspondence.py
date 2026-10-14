"""Check pinned source labels without adopting source anatomy or dimensions."""
from __future__ import annotations

import re


def audit_correspondence(catalog, audit, observed_audit_sha256, canonical_ids):
    if (catalog.get("authority") != "curated_reference_correspondence_not_geometry_adoption"
            or catalog.get("canonical_transform") is not None
            or catalog.get("fabrication_released") is not False):
        raise ValueError("Reference correspondence cannot adopt geometry or release fabrication")
    revision = catalog.get("source_revision")
    if not isinstance(revision, str) or not re.fullmatch(r"[0-9a-f]{40}", revision):
        raise ValueError("Missing immutable source revision")
    if (catalog.get("audit_sha256") != observed_audit_sha256
            or catalog.get("source_asset") != audit.get("asset_id")
            or catalog.get("source_sha256") != audit.get("sha256")):
        raise ValueError("Source or audit integrity mismatch")
    if (audit.get("coordinate_contract") != "glTF right-handed Y-up; linear units meters"
            or audit.get("anatomical_axis_registration") is not None
            or audit.get("anatomically_qualified") is not False
            or audit.get("fabrication_released") is not False):
        raise ValueError("Unknown native frame or unreviewed qualification")
    names = {}
    for node in audit["nodes"]:
        names.setdefault(node.get("name"), []).append(node)
    entries = catalog["entries"]
    ids = [row["bone_id"] for row in entries]
    if not ids or len(set(ids)) != len(ids) or not set(ids) <= set(canonical_ids):
        raise ValueError("Duplicate or noncanonical bone identity")
    used = set()
    rows = []
    for entry in entries:
        labels = entry["source_nodes"]
        if not labels or len(set(labels)) != len(labels) or not entry.get("mapping_basis"):
            raise ValueError("Missing or duplicate source-label correspondence")
        nodes = []
        for label in labels:
            matches = names.get(label, [])
            if len(matches) != 1 or label in used:
                raise ValueError("Missing, ambiguous or reused source node: " + str(label))
            node = matches[0]
            if type(node.get("mesh")) is not int or node["mesh"] < 0 or node.get("in_default_scene") is not True:
                raise ValueError("Mapped label has no inspected scene mesh")
            used.add(label)
            nodes.append({"name": label, "index": node["index"], "mesh": node["mesh"],
                          "triangles": node["triangles"]})
        rows.append({"bone_id": entry["bone_id"], "source_nodes": nodes,
                     "mapping_result": "label_correspondence_only",
                     "canonical_dimensions_adopted": False, "landmarks_qualified": False})
    unmapped = []
    for entry in catalog["unmapped_source_review"]:
        label = entry["source_node"]
        if len(names.get(label, [])) != 1 or label in used or not entry.get("reason"):
            raise ValueError("Variant source disposition is missing or mapped")
        unmapped.append(entry)
    return {"source_revision": revision, "source_asset": catalog["source_asset"],
            "source_sha256": catalog["source_sha256"], "audit_sha256": observed_audit_sha256,
            "source_units": "m", "source_frame": "native glTF scene",
            "matched_canonical_ids": len(rows), "matched_mesh_nodes": len(used),
            "canonical_transform": None, "unmapped_source_review": unmapped,
            "rows": rows, "software_integrity_pass": True,
            "anatomically_qualified": False, "fabrication_released": False}
