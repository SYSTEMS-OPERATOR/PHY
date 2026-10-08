"""Adult bone accounting for project-local Studio proxies, not BoneSpec adoption.

Inventory is independent of the renderer's meshes. Missing source records stay
missing. Neither grouped forms nor hardware inflate individual bone coverage.
"""
from __future__ import annotations

import ast
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
EXPECTED_REGIONS = {
    "skull": 22, "ear": 6, "hyoid": 1, "spine": 26, "thorax": 25,
    "shoulder_girdle": 4, "pelvis": 2, "upper_limb": 6,
    "lower_limb": 8, "hand": 54, "foot": 52,
}
CORE_REGIONS = {"spine", "thorax", "shoulder_girdle", "pelvis", "upper_limb", "lower_limb"}
ROLES = {"bone_proxy", "hardware", "support", "form", "envelope"}


def expected_bones():
    """206 stable adult identities, using all existing PHY ID spellings."""
    rows = []

    def add(code, name, region, side="midline"):
        rows.append({"bone_id": "BONE_" + code, "name": name, "region": region,
                     "side": side, "core_scope": region in CORE_REGIONS})

    def pair(code, name, region):
        for side in ("R", "L"):
            add(code + "_" + side, name, region, "right" if side == "R" else "left")

    for code in ("FRONTAL", "OCCIPITAL", "SPHENOID", "ETHMOID"):
        add(code, code.title(), "skull")
    pair("PAR", "Parietal", "skull"); pair("TEMP", "Temporal", "skull")
    for code in ("MAXILLA", "ZYGOMATIC", "NASAL", "LACRIMAL", "PALATINE", "INFERIOR_NASAL_CONCHA"):
        pair(code, code.replace("_", " ").title(), "skull")
    add("MANDIBLE", "Mandible", "skull"); add("VOMER", "Vomer", "skull")
    for code in ("MALLEUS", "INCUS", "STAPES"):
        pair(code, code.title(), "ear")
    add("HYOID", "Hyoid", "hyoid")
    for prefix, count in (("C", 7), ("T", 12), ("L", 5)):
        for level in range(1, count + 1):
            add(f"{prefix}{level}", f"{prefix}{level} vertebra", "spine")
    add("SACRUM", "Sacrum", "spine"); add("COCCYX", "Coccyx", "spine")
    for level in range(1, 13):
        pair(f"RIB{level}", f"Rib {level}", "thorax")
    add("STERNUM", "Sternum", "thorax")
    for code in ("CLAVICLE", "SCAPULA"):
        pair(code, code.title(), "shoulder_girdle")
    pair("HIP", "Adult hip bone", "pelvis")
    for code in ("HUMERUS", "RADIUS", "ULNA"):
        pair(code, code.title(), "upper_limb")
    for code in ("FEMUR", "PATELLA", "TIBIA", "FIBULA"):
        pair(code, code.title(), "lower_limb")
    for code in ("SCAPHOID", "LUNATE", "TRIQUETRUM", "PISIFORM", "TRAPEZIUM", "TRAPEZOID", "CAPITATE", "HAMATE"):
        pair(code, code.title(), "hand")
    for digit in range(1, 6):
        pair(f"META{digit}", f"Metacarpal {digit}", "hand")
        for segment in range(1, 3 if digit == 1 else 4):
            pair(f"PHAL_{digit}_{segment}", f"Hand digit {digit} phalanx {segment}", "hand")
    for code in ("TALUS", "CALCANEUS", "NAVICULAR", "CUBOID", "MEDIAL_CUNEIFORM", "INTERMEDIATE_CUNEIFORM", "LATERAL_CUNEIFORM"):
        pair(code, code.replace("_", " ").title(), "foot")
    for digit in range(1, 6):
        pair(f"MT{digit}", f"Metatarsal {digit}", "foot")
        for segment in range(1, 3 if digit == 1 else 4):
            pair(f"T_PHAL_{digit}_{segment}", f"Toe {digit} phalanx {segment}", "foot")
    return sorted(rows, key=lambda row: row["bone_id"])


def source_record_paths(root=ROOT):
    """Read IDs without importing legacy modules or filling unknown dimensions."""
    paths = {}
    for path in sorted((Path(root) / "skeleton/bones").glob("bone_*.py")):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
                 and isinstance(n.func, ast.Name) and n.func.id == "BoneSpec"]
        ids = [ast.literal_eval(k.value) for n in calls for k in n.keywords if k.arg == "unique_id"]
        if len(ids) != 1 or ids[0] in paths:
            raise ValueError("invalid or duplicated source bone ID: " + str(path))
        paths[ids[0]] = str(path.relative_to(root))
    return paths


def report_markdown(audit):
    lines = ["# PHY adult bone-equivalence coverage", "",
             "Status: adult 206 identity/distribution review only; dimensions/morphology unverified, physical evidence unmeasured, fabrication release false.", "",
             f"{audit['expected_bones']} adult identities: {audit['individual_bone_proxies']} individual project proxies, "
             f"{audit['grouped_bones']} grouped-form identities, {audit['unrepresented_bones']} unrepresented.",
             f"{audit['source_records_present']} existing canonical source records are unchanged; "
             f"{len(audit['missing_source_records'])} facial records remain missing.", "",
             "One adult fused hip, sacrum and coccyx each. Hardware, cartilage, teeth and support rails never increase bone coverage.", "",
             "| Region | Expected | Individual | Grouped | Unrepresented |",
             "| --- | ---: | ---: | ---: | ---: |"]
    lines += [f"| {r['region']} | {r['expected']} | {r['individual']} | {r['grouped']} | {r['unrepresented']} |" for r in audit["regions"]]
    lines += ["", "## Outstanding source records", ""]
    lines += ["- `"+key+"`" for key in audit["missing_source_records"]]
    lines += ["", "## Every identity and disposition", "",
              "All individual proxies use provisional project geometry. Count equivalence is not dimensional fidelity or manufacture approval.", "",
              "| Bone ID | Region | Representation | Mesh ID(s) | Source record |",
              "| --- | --- | --- | --- | --- |"]
    lines += [f"| {r['bone_id']} | {r['region']} | {r['representation']} | {', '.join(r['mesh_ids']) or '—'} | {r['source_record'] or 'MISSING'} |" for r in audit["bones"]]
    return "\n".join(lines)+"\n"


def bone_audit(parts, root=ROOT):
    """Return all 206 dispositions and reject invalid/ambiguous mesh mappings."""
    inventory = expected_bones()
    wanted = {row["bone_id"]: row for row in inventory}
    if dict(Counter(row["region"] for row in inventory)) != EXPECTED_REGIONS or len(wanted) != 206:
        raise ValueError("adult inventory contract is inconsistent")
    records = source_record_paths(root)
    unknown_records = set(records) - set(wanted)
    if unknown_records:
        raise ValueError("unknown source bone IDs: " + ", ".join(sorted(unknown_records)))
    individual, grouped, part_ids = {}, {}, set()
    region_names = {"spine": "spine", "thorax": "thorax", "shoulder_girdle": "shoulders",
                    "pelvis": "pelvis", "upper_limb": "arms", "lower_limb": "legs",
                    "skull": "head", "ear": "head", "hyoid": "head", "hand": "hands", "foot": "feet"}
    for part in parts:
        if part["id"] in part_ids:
            raise ValueError("duplicate mesh ID: " + part["id"])
        part_ids.add(part["id"])
        role = part.get("role")
        if role not in ROLES:
            raise ValueError("unclassified mesh: " + part["id"])
        bone_id = part.get("bone_id")
        groups = part.get("grouped_bone_ids", [])
        if role == "bone_proxy":
            if bone_id not in wanted or groups:
                raise ValueError("invalid individual bone mapping: " + part["id"])
            if bone_id in individual:
                raise ValueError("duplicate individual bone coverage: " + bone_id)
            if part["region"] != region_names[wanted[bone_id]["region"]]:
                raise ValueError("bone mesh is mapped to the wrong region")
            if not part.get("geometry_inputs") or part.get("dimensional_fidelity") != "unverified" or part.get("physical_evidence") != "unmeasured":
                raise ValueError("bone proxy lacks explicit provisional provenance")
            individual[bone_id] = part["id"]
        elif bone_id is not None:
            raise ValueError("non-bone mesh carries an individual bone ID")
        if groups and role != "form":
            raise ValueError("only grouped form proxies may carry group correspondence")
        for key in groups:
            if key not in wanted or key in grouped:
                raise ValueError("invalid or duplicate grouped bone mapping: " + key)
            grouped[key] = part["id"]
    if set(individual) & set(grouped):
        raise ValueError("bone is both individually and collectively represented")
    rows = []
    for row in inventory:
        key = row["bone_id"]
        mesh = next((p for p in parts if p["id"] == individual.get(key)), None)
        rows.append(dict(row, source_record=records.get(key),
                         source_record_status="present" if key in records else "missing",
                         representation="individual_project_proxy" if key in individual else
                         "grouped_form_proxy" if key in grouped else "unrepresented",
                         mesh_ids=[individual[key]] if key in individual else [grouped[key]] if key in grouped else [],
                         geometry_inputs=mesh.get("geometry_inputs", []) if mesh else [],
                         dimensional_fidelity="unverified", physical_evidence="unmeasured"))
    regions = []
    for region, expected in EXPECTED_REGIONS.items():
        subset = [row for row in rows if row["region"] == region]
        regions.append({"region": region, "expected": expected,
                        "individual": sum(r["representation"] == "individual_project_proxy" for r in subset),
                        "grouped": sum(r["representation"] == "grouped_form_proxy" for r in subset),
                        "unrepresented": sum(r["representation"] == "unrepresented" for r in subset)})
    return {"inventory_version": "PHY_ADULT_206_V1", "expected_bones": 206,
            "source_records_present": len(records),
            "missing_source_records": sorted(set(wanted) - set(records)),
            "individual_bone_proxies": len(individual), "grouped_bones": len(grouped),
            "unrepresented_bones": 206 - len(individual) - len(grouped),
            "missing_core_bones": [r["bone_id"] for r in rows if r["core_scope"] and r["representation"] != "individual_project_proxy"],
            "missing_hand_bones": [r["bone_id"] for r in rows if r["region"] == "hand" and r["representation"] != "individual_project_proxy"],
            "missing_foot_bones": [r["bone_id"] for r in rows if r["region"] == "foot" and r["representation"] != "individual_project_proxy"],
            "missing_skull_bones": [r["bone_id"] for r in rows if r["region"] == "skull" and r["representation"] != "individual_project_proxy"],
            "missing_ear_bones": [r["bone_id"] for r in rows if r["region"] == "ear" and r["representation"] != "individual_project_proxy"],
            "missing_hyoid_bones": [r["bone_id"] for r in rows if r["region"] == "hyoid" and r["representation"] != "individual_project_proxy"],
            "mesh_roles": dict(sorted(Counter(p["role"] for p in parts).items())),
            "regions": regions, "bones": rows,
            "bone_distribution_complete": len(individual) == 206,
            "dimensional_fidelity_verified": False, "fabrication_released": False,
            "physical_evidence_complete": False}
