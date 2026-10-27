"""Bounded qualitative craniofacial catalog contract, with no geometry imports."""
from datetime import date
import json
from pathlib import Path
import re
from urllib.parse import urlsplit

from skeleton.anatomical_inventory import ROOT, INVENTORY_VERSION, expected_bones

CATALOG_PATH = "references/craniofacial_relationships.json"
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
    json.dumps(catalog, sort_keys=True, allow_nan=False)


def load_catalog(root=ROOT):
    catalog = json.loads((Path(root) / CATALOG_PATH).read_text(encoding="utf-8"))
    validate_catalog(catalog)
    return catalog
