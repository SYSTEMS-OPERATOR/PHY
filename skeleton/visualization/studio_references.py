"""Explicit, offline admission of two hash-pinned PHY-REF native previews.

No acquisition, large original read, registration or anatomical measurement.
The existing PHY-REF static inspector is loaded only after its hash is checked.
"""
from __future__ import annotations

import base64
import hashlib
import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
LOCK_PATH = Path("studio/reference_lock.json")
PREVIEWS = {"NIH_3DPX_020984": "pelvis-preview.glb", "NIH_3DPX_020986": "skin-preview.glb"}


def load_native_references(reference_root, root=ROOT):
    """Fail closed if any selected file differs; never silently fall back."""
    reference_root = Path(reference_root).resolve()
    lock = json.loads((Path(root) / LOCK_PATH).read_text())
    if (lock["version"] != "PHY_STUDIO_NATIVE_REFERENCES_V1"
            or lock["repository"] != "SYSTEMS-OPERATOR/PHY-REF"
            or lock["canon_effect"] != "none" or lock["fabrication_released"] is not False):
        raise ValueError("Unsupported reference lock")
    raw_files = {}
    for relative, pin in lock["files"].items():
        path = (reference_root / relative).resolve()
        if not path.is_relative_to(reference_root):
            raise ValueError("Reference path escapes root")
        if not path.is_file() or path.stat().st_size != pin["bytes"]:
            raise ValueError("Missing or wrong-size pinned reference: " + relative)
        raw = path.read_bytes()
        if hashlib.sha256(raw).hexdigest() != pin["sha256"]:
            raise ValueError("Reference hash mismatch: " + relative)
        raw_files[relative] = raw
    spec = importlib.util.spec_from_file_location("_phy_studio_reference_glb", reference_root / "phy_ref/glb.py")
    inspector = importlib.util.module_from_spec(spec); spec.loader.exec_module(inspector)
    catalog = inspector.strict_json(raw_files["sources/reference_assets.json"])
    derivatives = inspector.strict_json(raw_files["assets/nih/derived/derivative_manifest.json"])
    packet = inspector.strict_json(raw_files["reviews/pelvis_left_landmarks.json"])
    if (packet["accepted_landmarks"] is not None or packet["canonical_transform"] is not None
            or packet["fabrication_released"] is not False):
        raise ValueError("Landmark packet must remain unaccepted")
    models = []
    for ident, filename in PREVIEWS.items():
        source = next(a for a in catalog["assets"] if a["asset_id"] == ident)
        derivative = next(a for a in derivatives["derivatives"] if a["asset_id"] == ident)
        path = "assets/nih/derived/" + filename
        if (source["license"] != "CC-BY-4.0" or source["canonical_transform"] is not None
                or source["anatomically_qualified"] is not False or source["fabrication_released"] is not False
                or derivative["source_sha256"] != source["sha256"]
                or derivative["preview_sha256"] != lock["files"][path]["sha256"]
                or derivative["canonical_transform"] is not None
                or derivative["coordinate_frame"] != "source world Y-up, unchanged"
                or derivative["linear_units"] != "m" or derivative["fabrication_released"] is not False):
            raise ValueError("Native reference lineage/qualification mismatch")
        glb = inspector.GLB(raw_files[path]); audited = glb.inspect()
        if audited["scene_triangles"] != derivative["preview_triangles"]:
            raise ValueError("Preview triangle count mismatch")
        lineage = {n["name"]: n for n in derivative["nodes"]}
        parts = []
        for node in audited["nodes"]:
            if not node["in_default_scene"] or node["mesh"] is None:
                continue
            native = glb.doc["nodes"][node["index"]]
            parent = lineage[node["name"]]
            if (native["extras"]["source_node"] != parent["source_node"]
                    or native["extras"]["source_sha256"] != source["sha256"]
                    or node["triangles"] != parent["preview_triangles"]):
                raise ValueError("Preview node lineage mismatch")
            parts.append({"id": node["name"], "name": node["name"], "region": "native_reference",
                          "preview_node": node["index"], "source_node": parent["source_node"],
                          "triangle_count": node["triangles"], "bounds_m": node["bounds"],
                          "bounds_mm": {k: [v*1000 for v in a] for k, a in node["bounds"].items()},
                          "authority": "NIH/HRA native-frame decimated preview; anatomy/metric fidelity unqualified",
                          "source": source["entry_url"]})
        models.append({"id": "NIH_PELVIS" if ident.endswith("020984") else "NIH_SKIN",
                       "name": source["title"], "kind": "nih_native_preview", "units": "m",
                       "frame": "source world Y-up, unchanged", "parts": parts,
                       "bounds_m": audited["bounds_m"], "source": source, "derivative": derivative,
                       "reference_revision": lock["revision"], "preview_path": path,
                       "glb_base64": base64.b64encode(raw_files[path]).decode("ascii"),
                       "canonical_transform": None, "fabrication_released": False,
                       "limitations": [derivative["qualification"],
                           "Native meter coordinates; display conversion is 1000 mm/m only. No anatomical axis mapping or stature normalization.",
                           "Anatomy, metric calibration, joints, mass/inertia and fabrication remain unqualified.",
                           "Skin is exterior visual context and supplies no bone dimensions or tissue mechanics." if ident.endswith("020986") else
                           "14 tissue meshes represent four fused adult bones; source labels are not accepted canonical correspondence."]})
    evidence = {"version": lock["version"], "repository": lock["repository"], "revision": lock["revision"],
                "review_status": lock["review_status"], "files": lock["files"],
                "landmark_packet": packet,
                "landmark_packet_sha256": lock["files"]["reviews/pelvis_left_landmarks.json"]["sha256"],
                "attribution": raw_files["assets/nih/ATTRIBUTION.md"].decode(),
                "canon_effect": "none", "fabrication_released": False}
    return models, evidence, {p: raw_files[p] for p in raw_files if p.endswith(".glb")}
