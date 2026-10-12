"""Offline qualitative craniofacial relationships and armature boundary review.

Participation in a region is not a bone adjacency, assembly parent or interface.
This catalog never supplies mechanical transforms or facial canon coordinates.
"""
from __future__ import annotations

from copy import deepcopy
from datetime import date
import json
import math
from pathlib import Path
import re
from urllib.parse import urlsplit

from skeleton.anatomical_inventory import ROOT, INVENTORY_VERSION, expected_bones
from skeleton.exporters.exporter_agent import json_bytes, sha256
from skeleton.exporters.publication import publish_files
from skeleton.validation.engineering_audit import engineering_audit
from skeleton.visualization.studio_model import reference_statistics

CATALOG_PATH = "references/craniofacial_relationships.json"
PROFILE_PATH = "PROJECTS/PHY_F28/profiles/f28.json"
AUDIT_VERSION = "1.0.0"
REGION_IDS = {"orbit_r", "orbit_l", "nasal_bridge", "nasal_septum", "hard_palate",
              "mandibular_unit", "zygomatic_arches"}


def validate_catalog(catalog):
    """Check the bounded, qualitative contract; do not verify publisher content.

    A perception citation cannot qualify anatomy. Numeric/spatial engineering
    definitions need a different reviewed contract and are rejected here.
    """
    def fields(value, names, label):
        if not isinstance(value, dict) or set(value) != set(names.split()):
            raise ValueError("Invalid fields: " + label)

    def text(value, label, pattern=None):
        if (not isinstance(value, str) or not value or value != value.strip()
                or (pattern and not re.fullmatch(pattern, value))):
            raise ValueError("Invalid text: " + label)

    def strings(value, label):
        if not isinstance(value, list) or not value:
            raise ValueError("Expected nonempty list: " + label)
        for item in value:
            text(item, label)
        if len(set(value)) != len(value):
            raise ValueError("Duplicate declaration: " + label)

    fields(catalog, "catalog_version inventory_version scope authority sources regions articulations", "catalog")
    if (catalog["catalog_version"] != "1.0.0" or catalog["inventory_version"] != INVENTORY_VERSION
            or catalog["scope"] != "selected_craniofacial_regions_and_bilateral_tmjs"
            or catalog["authority"] != "curated_qualitative_reference_not_canon_or_mechanical_design"):
        raise ValueError("Unsupported craniofacial catalog contract")
    sources = {}
    if not isinstance(catalog["sources"], list) or not catalog["sources"]:
        raise ValueError("Missing source declarations")
    for source in catalog["sources"]:
        fields(source, "source_id kind title url reviewed_on supports review_scope", "source")
        text(source["source_id"], "source_id", r"[A-Z][A-Z0-9_]*")
        if source["source_id"] in sources:
            raise ValueError("Duplicate source ID")
        for key in ("title", "url", "reviewed_on", "review_scope"):
            text(source[key], key)
        text(source["kind"], "source kind")
        url = urlsplit(source["url"])
        if url.scheme != "https" or not url.netloc or url.username or url.password:
            raise ValueError("Expected public HTTPS reference")
        if date.fromisoformat(source["reviewed_on"]).isoformat() != source["reviewed_on"]:
            raise ValueError("Expected ISO review date")
        strings(source["supports"], "source scopes")
        allowed = {"anatomical_reference": {"structural_participation", "articulation_identity", "qualitative_motion"},
                   "perception_context": {"perception_context"}}
        if source["kind"] not in allowed or not set(source["supports"]) <= allowed[source["kind"]]:
            raise ValueError("Invalid evidence kind or scope")
        sources[source["source_id"]] = source

    inventory = {row["bone_id"]: row for row in expected_bones()}

    def anatomy_source(row, scopes):
        text(row["source_id"], "source_id")
        source = sources.get(row["source_id"])
        if (source is None or source["kind"] != "anatomical_reference"
                or not scopes <= set(source["supports"])):
            raise ValueError("Anatomical claim lacks a source with the required scope")
        for key in ("name", "locator"):
            text(row[key], key)
        strings(row["evidence_needed"], "evidence_needed")
        if row["side"] not in ("right", "left", "midline", "bilateral"):
            raise ValueError("Invalid anatomical side")

    if not isinstance(catalog["regions"], list) or len(catalog["regions"]) != len(REGION_IDS):
        raise ValueError("Incomplete selected region set")
    seen = set()
    for region in catalog["regions"]:
        fields(region, "region_id name side source_id locator contributions external_overlap evidence_needed spatial_definition", "region")
        uid = region["region_id"]
        text(uid, "region_id", r"[a-z][a-z0-9_]*")
        if uid not in REGION_IDS or uid in seen:
            raise ValueError("Unknown or duplicate region ID")
        seen.add(uid)
        anatomy_source(region, {"structural_participation"})
        if region["spatial_definition"] is not None:
            raise ValueError("Spatial definitions are unqualified in this catalog")
        strings(region["external_overlap"], "external_overlap")
        if not isinstance(region["contributions"], list) or not region["contributions"]:
            raise ValueError("Missing structural participants")
        bone_ids = set()
        for contribution in region["contributions"]:
            fields(contribution, "bone_id role", "contribution")
            text(contribution["bone_id"], "bone_id")
            text(contribution["role"], "role")
            bone = inventory.get(contribution["bone_id"])
            if bone is None or bone["region"] != "skull" or bone["bone_id"] in bone_ids:
                raise ValueError("Unknown, non-skull or duplicate structural participant")
            bone_ids.add(bone["bone_id"])
            if region["side"] in ("right", "left") and bone["side"] not in ("midline", region["side"]):
                raise ValueError("Opposite-side structural participant")
        required_side = {"orbit_r": "right", "orbit_l": "left", "zygomatic_arches": "bilateral"}.get(uid, "midline")
        if region["side"] != required_side:
            raise ValueError("Region side does not match its ID")

    if not isinstance(catalog["articulations"], list) or len(catalog["articulations"]) != 2:
        raise ValueError("Expected both TMJ articulations")
    seen = set()
    for joint in catalog["articulations"]:
        fields(joint, "articulation_id name side source_id locator bone_ids anatomical_type motion_components non_bone_components evidence_needed engineering_definition", "articulation")
        uid = joint["articulation_id"]
        text(uid, "articulation_id", r"[a-z][a-z0-9_]*")
        if uid not in ("tmj_r", "tmj_l") or uid in seen:
            raise ValueError("Unknown or duplicate articulation ID")
        seen.add(uid)
        anatomy_source(joint, {"articulation_identity", "qualitative_motion"})
        strings(joint["bone_ids"], "articulation bone_ids")
        suffix = uid[-1].upper()
        if (set(joint["bone_ids"]) != {"BONE_MANDIBLE", "BONE_TEMP_" + suffix}
                or joint["side"] != ("right" if suffix == "R" else "left")):
            raise ValueError("TMJs must share one mandible and use the ipsilateral temporal bone")
        strings(joint["motion_components"], "motion_components")
        strings(joint["non_bone_components"], "non_bone_components")
        if (joint["anatomical_type"] != "synovial"
                or set(joint["motion_components"]) != {"rotation", "translation"}
                or set(joint["non_bone_components"]) != {"articular_disc"}):
            raise ValueError("TMJ qualitative motion and disc cannot be reduced to a fixed hinge")
        if joint["engineering_definition"] is not None:
            raise ValueError("Mechanical definitions are unqualified in this catalog")
    json_bytes(catalog)


def load_catalog(root=ROOT):
    catalog = json.loads((Path(root) / CATALOG_PATH).read_text(encoding="utf-8"))
    validate_catalog(catalog)
    return catalog


def craniofacial_audit(skeleton, model, *, catalog=None, root=ROOT):
    """Join regional declarations to the existing independent evidence audit.

    Missing canonical records or proxies are reportable, not fabricated. Neither
    input snapshot is mutated; catalog relations never fill BoneSpec connections.
    """
    root = Path(root)
    catalog = deepcopy(load_catalog(root) if catalog is None else catalog)
    validate_catalog(catalog)
    catalog_hash = sha256(json_bytes(catalog))
    if (model.get("canon_effect") != "none" or model.get("fabrication_released") is not False
            or model.get("physical_evidence_complete") is not False):
        raise ValueError("Expected an unqualified reference-body inspection model")
    observed = engineering_audit(skeleton, model["parts"], root=root)
    rows = {row["bone_id"]: row for row in observed["bones"]}
    ids = {c["bone_id"] for region in catalog["regions"] for c in region["contributions"]}
    ids.update(uid for joint in catalog["articulations"] for uid in joint["bone_ids"])
    bones = [rows[uid] for uid in sorted(ids)]
    ref, scale, height = model["reference"], model["scale_from_mean"], model["height_mm"]
    if json_bytes(ref) != json_bytes(reference_statistics()):
        raise ValueError("Head guide reference differs from the verified local cohort")
    for value in (height, scale, ref["statistics"]["stature"]["mean_mm"]):
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError("Invalid head guide scale")
    if not math.isclose(scale, height / ref["statistics"]["stature"]["mean_mm"], rel_tol=1e-12):
        raise ValueError("Head guide scale differs from the model's reference stature")
    profile_bytes = (root / PROFILE_PATH).read_bytes()
    profile = json.loads(profile_bytes)
    head_height = profile["design_choices_mm"]["head_height"]
    if type(head_height) not in (int, float) or not math.isfinite(head_height) or head_height <= 0:
        raise ValueError("Invalid project head-height choice")
    guide_axes = []
    for field, axis in (("headbreadth", "x"), ("headlength", "y")):
        stats = ref["statistics"][field]
        mean = stats["mean_mm"]
        if type(mean) not in (int, float) or not math.isfinite(mean) or mean <= 0:
            raise ValueError("Invalid external head statistic")
        guide_axes.append({"axis": axis, "field": field, "basis": "external_cohort_marginal_mean",
                           "statistics": deepcopy(stats), "guide_value_mm": round(mean * scale, 6)})
    guide_axes.append({"axis": "z", "field": "head_height", "basis": "project_design_choice",
                       "source": PROFILE_PATH + "#/design_choices_mm/head_height",
                       "base_value_mm": head_height, "guide_value_mm": round(head_height * scale, 6)})
    regions = sorted(catalog["regions"], key=lambda row: row["region_id"])
    for region in regions:
        region["contributions"].sort(key=lambda row: row["bone_id"])
        region["missing_canonical_records"] = sorted(c["bone_id"] for c in region["contributions"]
            if rows[c["bone_id"]]["canonical_record"]["status"] != "present")
        region["missing_display_proxies"] = sorted(c["bone_id"] for c in region["contributions"]
            if rows[c["bone_id"]]["display"]["representation"] != "individual_project_proxy")
    joints = sorted(catalog["articulations"], key=lambda row: row["articulation_id"])
    for joint in joints:
        joint["bone_ids"].sort()
        joint["missing_canonical_records"] = [uid for uid in joint["bone_ids"]
            if rows[uid]["canonical_record"]["status"] != "present"]
        joint["missing_display_proxies"] = [uid for uid in joint["bone_ids"]
            if rows[uid]["display"]["representation"] != "individual_project_proxy"]
    report = {
        "audit_version": AUDIT_VERSION, "scope": catalog["scope"], "catalog_version": catalog["catalog_version"],
        "inventory_version": INVENTORY_VERSION,
        "inputs": {**observed["inputs"], "catalog_sha256": catalog_hash,
                   "head_profile_sha256": sha256(profile_bytes)},
        "sources": sorted(catalog["sources"], key=lambda row: row["source_id"]),
        "regions": regions, "articulations": joints, "bones": bones,
        "mandibular_system": {"articulation_ids": [joint["articulation_id"] for joint in joints],
                              "shared_moving_bone_id": "BONE_MANDIBLE", "engineering_definition": None},
        "head_guide_context": {"model_id": model["id"], "height_mm": height, "scale_from_mean": scale,
                               "source_id": ref["source"], "population": ref["population"],
                               "cohort_n": ref["n"], "age_years": ref["age_years"],
                               "snapshot_sha256": ref["snapshot_sha256"], "axes": guide_axes,
                               "status": "external_envelope_inputs_not_bone_or_facial_landmark_measurements",
                               "canon_effect": "none"},
        "counts": {"structural_regions": len(regions), "qualitative_articulations": len(joints),
                   "participating_bone_identities": len(bones),
                   "canonical_records_present": sum(row["canonical_record"]["status"] == "present" for row in bones),
                   "individual_display_proxies": sum(row["display"]["representation"] == "individual_project_proxy" for row in bones),
                   "spatially_defined_regions": 0, "mechanically_defined_articulations": 0},
        "qualification": {**observed["qualification"], "external_feature_fit": "unresolved",
                          "canonical_face_landmarks": "unbound", "facial_ratios_adopted": False},
        "limitations": ["Selected structures only; not a complete skull contact or suture graph.",
                        "Region participation does not imply pairwise contact, a rigid parent or a mount.",
                        "Anatomical TMJ declarations do not resolve legacy temporal references or qualify mechanisms.",
                        "External head means and a project head-height choice do not locate orbital centers or define bone-to-skin offsets.",
                        "Perception research is adjacent design context; it does not qualify skeletal dimensions.",
                        "Source scopes are curated declarations; this offline audit does not verify publisher text."],
    }
    json_bytes(report)
    return report


def report_markdown(report):
    def cell(value):
        return str(value).replace("|", "\\|").replace("\n", " ").replace("\r", " ")

    lines = ["# Craniofacial armature boundary review", "", "Qualitative anatomy and evidence gaps; fabrication release remains false.", "",
             "Region participation is not pairwise adjacency, an assembly parent or a mechanical interface.", "",
             "| Observation | Count |", "| --- | ---: |"]
    lines += [f"| {cell(key)} | {value} |" for key, value in report["counts"].items()]
    lines += ["", "## Structural regions", "", "External overlap and evidence-needed entries are engineering review questions; they do not qualify feature fit.", ""]
    for region in report["regions"]:
        lines += ["### " + cell(region["name"]), "", f"Source: `{region['source_id']}` / {cell(region['locator'])}.", "",
                  "| Canonical bone ID | Anatomical contribution | Studio mesh IDs |", "| --- | --- | --- |"]
        by_id = {row["bone_id"]: row for row in report["bones"]}
        lines += [f"| {c['bone_id']} | {cell(c['role'])} | {cell(', '.join(by_id[c['bone_id']]['display']['mesh_ids']))} |"
                  for c in region["contributions"]]
        lines += ["", "Overlap to review: " + cell("; ".join(region["external_overlap"])) + ".", "",
                  "Evidence needed: " + cell("; ".join(region["evidence_needed"])) + ".", "",
                  "Spatial definition: unresolved. Missing canonical records: " + cell(", ".join(region["missing_canonical_records"]) or "none")
                  + "; missing display proxies: " + cell(", ".join(region["missing_display_proxies"]) or "none") + ".", ""]
    lines += ["## Mandibular system", "", "Both TMJ sites share `BONE_MANDIBLE`; two sites do not create two independently positioned jaws.", "",
              "| Articulation | Bone IDs | Anatomical motion | Engineering definition |", "| --- | --- | --- | --- |"]
    lines += [f"| {j['articulation_id']} | {', '.join(j['bone_ids'])} | {', '.join(j['motion_components'])}; articular disc | unresolved |"
              for j in report["articulations"]]
    lines += ["", "Anatomical motion includes rotation and translation. Axes, trajectory, ranges, disc/substitute surfaces and mechanical acceptance remain unknown.", "",
              "## Existing head guide inputs", "", cell(report["head_guide_context"]["population"]), "",
              "Global stature scaling is a display design transform. Marginal means are not a measured individual or facial landmark map.", "",
              "| Axis | Input | Basis | Guide value (mm) |", "| --- | --- | --- | ---: |"]
    lines += [f"| {a['axis']} | {a['field']} | {a['basis']} | {a['guide_value_mm']} |" for a in report["head_guide_context"]["axes"]]
    lines += ["", "## Canonical and display evidence", "", "| Bone ID | Canonical record | Declared dimensions | Canonical geometry | Existing record gate |", "| --- | --- | --- | --- | --- |"]
    lines += [f"| {r['bone_id']} | {r['canonical_record']['status']} | {r['dimensions']['status']} | "
              f"{(r['canonical_geometry'] or {}).get('status', 'missing')} | {r['record_validation']['status']} |" for r in report["bones"]]
    lines += ["", "## Reference scope", "", "| Source | Kind | Locator / reviewed scope |", "| --- | --- | --- |"]
    lines += [f"| [{s['source_id']}]({s['url']}) | {s['kind']} | {cell(s['review_scope'])} |" for s in report["sources"]]
    lines += ["", "## Limits", ""] + ["- " + cell(value) for value in report["limitations"]]
    return "\n".join(lines) + "\n"


def write_reports(report, output_root):
    """Use the existing rollback publisher and manifest hashes for review only."""
    if (report["qualification"]["fabrication_released"] is not False
            or report["qualification"]["facial_ratios_adopted"] is not False
            or report["qualification"]["mechanical_transforms"] != "unresolved"
            or any(r["spatial_definition"] is not None for r in report["regions"])
            or any(j["engineering_definition"] is not None for j in report["articulations"])
            or report["mandibular_system"]["engineering_definition"] is not None):
        raise ValueError("Only an unqualified craniofacial review may be published")
    directory = Path(output_root) / "reports"
    json_path, md_path = directory / "craniofacial_audit.json", directory / "craniofacial_audit.md"
    manifest_path = directory / "craniofacial_audit_manifest.json"
    payloads = {json_path: json_bytes(report), md_path: report_markdown(report).encode("utf-8")}
    payloads[manifest_path] = json_bytes({"audit_version": AUDIT_VERSION, "scope": report["scope"],
                                      "inputs": report["inputs"], "fabrication_released": False,
                                      "files": {p.name: {"sha256": sha256(content)} for p, content in payloads.items()}})
    publish_files(payloads, manifest_path)
    return {"json": str(json_path), "markdown": str(md_path), "manifest": str(manifest_path)}
