"""Individual foot identity/placement proxies, never adopted osteometry.

Grounded static display only. Surfaces, arches, load paths, ankle/toe mechanics
and measured joints remain unresolved; canonical BoneSpec records are untouched.
"""
from __future__ import annotations

import math

TARSAL_PARENTS = {
    "TALUS": "TIBIA", "CALCANEUS": "TALUS", "NAVICULAR": "TALUS",
    "CUBOID": "CALCANEUS", "MEDIAL_CUNEIFORM": "NAVICULAR",
    "INTERMEDIATE_CUNEIFORM": "NAVICULAR", "LATERAL_CUNEIFORM": "NAVICULAR",
}
DIGIT_ROOTS = {"1": "MEDIAL_CUNEIFORM", "2": "INTERMEDIATE_CUNEIFORM",
               "3": "LATERAL_CUNEIFORM", "4": "CUBOID", "5": "CUBOID"}
POINTER = "PROJECTS/PHY_F28/profiles/f28.json#/foot_proxy_layout"


def validate_foot_layout(layout):
    """Fail closed on missing IDs, reversed rows/chains and invalid sections."""
    def number(value, positive=False):
        return type(value) in (int, float) and math.isfinite(value) and (not positive or value > 0)

    def vector(value, positive=False):
        return isinstance(value, list) and len(value) == 3 and all(number(v, positive) for v in value)

    if (not isinstance(layout, dict) or layout.get("revision") != "FOOT_V1"
            or not isinstance(layout.get("tarsals"), dict) or set(layout["tarsals"]) != set(TARSAL_PARENTS)
            or not isinstance(layout.get("digits"), dict) or set(layout["digits"]) != set(DIGIT_ROOTS)):
        raise ValueError("invalid foot identity/layout contract")
    if not number(layout.get("joint_gap_mm"), True):
        raise ValueError("invalid foot display gap")
    if not number(layout.get("heel_behind_ankle_fraction")) or not 0 < layout["heel_behind_ankle_fraction"] < .5:
        raise ValueError("invalid foot heel/ankle offset")
    tarsals = layout["tarsals"]
    for code, item in tarsals.items():
        if (not isinstance(item, dict) or not vector(item.get("center"))
                or not vector(item.get("radii"), True)):
            raise ValueError("invalid tarsal placement/section: " + code)
        x, y, z = item["center"]
        rx, ry, rz = item["radii"]
        if not -.5 <= x-rx < x+rx <= .5 or not 0 <= y-ry < y+ry <= 1 or z < rz:
            raise ValueError("tarsal leaves the display envelope/floor")
    heel = tarsals["CALCANEUS"]
    if not math.isclose(heel["center"][1], heel["radii"][1], abs_tol=1e-9) or not math.isclose(
            heel["center"][2], heel["radii"][2], abs_tol=1e-9):
        raise ValueError("calcaneus must meet the heel and floor display datums")
    if tarsals["TALUS"]["center"][2] <= max(v["center"][2] for k, v in tarsals.items() if k != "TALUS"):
        raise ValueError("talus must be the superior tarsal")
    for anterior, posterior in (("NAVICULAR", "TALUS"), ("CUBOID", "CALCANEUS"),
                               ("MEDIAL_CUNEIFORM", "NAVICULAR"),
                               ("INTERMEDIATE_CUNEIFORM", "NAVICULAR"),
                               ("LATERAL_CUNEIFORM", "NAVICULAR")):
        if tarsals[anterior]["center"][1] <= tarsals[posterior]["center"][1]:
            raise ValueError("foot tarsal anterior/posterior order is inconsistent")
    medial_order = ("MEDIAL_CUNEIFORM", "INTERMEDIATE_CUNEIFORM", "LATERAL_CUNEIFORM", "CUBOID")
    if not all(tarsals[a]["center"][0] < tarsals[b]["center"][0] for a, b in zip(medial_order, medial_order[1:])):
        raise ValueError("foot tarsal medial/lateral order is inconsistent")
    for digit, root in DIGIT_ROOTS.items():
        item = layout["digits"][digit]
        count = 4 if digit == "1" else 5  # MT + two/three phalanges + end station.
        if (not isinstance(item, dict) or item.get("root") != root
                or not isinstance(item.get("stations"), list) or len(item["stations"]) != count
                or not all(vector(v) for v in item["stations"])
                or not isinstance(item.get("radii_mm"), list) or len(item["radii_mm"]) != count-1
                or not all(number(v, True) for v in item["radii_mm"])):
            raise ValueError("invalid foot digit chain/section: " + digit)
        if (not all(-.5 < x < .5 and 0 < y <= 1 and z > 0 for x, y, z in item["stations"])
                or not all(a[1] < b[1] for a, b in zip(item["stations"], item["stations"][1:]))):
            raise ValueError("invalid ordered foot digit stations")
        for i, radius in enumerate(item["radii_mm"]):
            if min(v[2] for v in item["stations"][i:i+2]) <= radius:
                raise ValueError("foot section reaches below the display floor")
    digits = layout["digits"]
    for index in (0, -1):
        if not all(digits[str(a)]["stations"][index][0] < digits[str(a+1)]["stations"][index][0] for a in range(1, 5)):
            raise ValueError("foot digits must run medial hallux to lateral fifth toe")
    if digits["1"]["radii_mm"][0] <= max(digits[str(i)]["radii_mm"][0] for i in range(2, 6)):
        raise ValueError("first metatarsal display section must be the thickest")


def build_foot(part, ankle, floor, length, breadth, scale, layout):
    """Emit exactly 26 right-foot identities; core's common mirror emits left."""
    from . import studio_model as sm
    from .bone_equivalence import expected_bones
    validate_foot_layout(layout)
    names = {r["bone_id"]: r["name"] for r in expected_bones()}
    origin = [ankle[0], ankle[1]-length*layout["heel_behind_ankle_fraction"], floor[2]]
    layout_hash, anchors, mt_lengths = sm.digest(layout), {}, {}

    def point(values):
        return sm.add(origin, [values[0]*breadth, values[1]*length, values[2]*scale])

    def emit(code, mesh, key, **details):
        part(code.lower()+"_R", names["BONE_"+code+"_R"]+" / provisional foot proxy",
             "feet", "redwood", mesh, source=POINTER, role="bone_proxy",
             bone_id="BONE_"+code+"_R", geometry_inputs=[POINTER+"/"+key,
             POINTER+"/heel_behind_ankle_fraction", POINTER+"/joint_gap_mm",
             "model#/reference/statistics/footlength", "model#/reference/statistics/footbreadthhorizontal",
             "model#/landmarks/ankle_R", "model#/landmarks/floor", "model#/scale_from_mean"],
             foot_layout_sha256=layout_hash, foot_frame_side="R",
             dimensional_fidelity="unverified", physical_evidence="unmeasured",
             attachment_status="partial topology only; articular surfaces, joints, arches and load paths unresolved",
             **details)

    for code, item in layout["tarsals"].items():
        center = point(item["center"])
        radii = [item["radii"][0]*breadth, item["radii"][1]*length, item["radii"][2]*scale]
        anchors["BONE_"+code+"_R"] = center
        emit(code, sm.ellipsoid(center, radii), "tarsals/"+code, center_mm=center,
             topology_parent="BONE_"+TARSAL_PARENTS[code]+"_R")

    gap = layout["joint_gap_mm"]*scale
    for digit, item in layout["digits"].items():
        points = [point(v) for v in item["stations"]]
        codes = ["MT"+digit] + [f"T_PHAL_{digit}_{i}" for i in range(1, len(points)-1)]
        for i, (code, a, b, radius) in enumerate(zip(codes, points, points[1:], item["radii_mm"])):
            distance = math.dist(a, b)
            if distance <= gap:
                raise ValueError("foot display gap consumes a bone segment")
            axis = sm.mul(sm.sub(b, a), 1/distance)
            endpoints = [sm.add(a, sm.mul(axis, gap/2)), sm.add(b, sm.mul(axis, -gap/2))]
            center = sm.mul(sm.add(a, b), .5)
            anchors["BONE_"+code+"_R"] = center
            if i == 0:
                mt_lengths[digit] = distance
            parent = item["root"] if i == 0 else codes[i-1]
            emit(code, sm.tube(endpoints, radius*scale), "digits/"+digit,
                 digit_index=int(digit), chain_index=i, center_mm=center,
                 joint_stations_mm=[a, b], endpoints_mm=endpoints,
                 center_distance_mm=distance-gap, display_gap_mm=gap,
                 topology_parent="BONE_"+parent+"_R",
                 topology_child="BONE_"+codes[i+1]+"_R" if i+1 < len(codes) else None)
    if mt_lengths["2"] <= max(v for k, v in mt_lengths.items() if k != "2"):
        raise ValueError("second metatarsal display station must be the longest")
    return anchors
