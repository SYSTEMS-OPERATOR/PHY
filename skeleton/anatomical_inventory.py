"""Shared adult anatomical identity inventory, independent of display geometry.

This moves the existing PHY_ADULT_206_V1 contract without changing its IDs,
regional accounting, source records or anatomical qualification.
"""
from __future__ import annotations

import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EXPECTED_REGIONS = {
    "skull": 22, "ear": 6, "hyoid": 1, "spine": 26, "thorax": 25,
    "shoulder_girdle": 4, "pelvis": 2, "upper_limb": 6,
    "lower_limb": 8, "hand": 54, "foot": 52,
}
CORE_REGIONS = {"spine", "thorax", "shoulder_girdle", "pelvis", "upper_limb", "lower_limb"}
INVENTORY_VERSION = "PHY_ADULT_206_V1"


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
