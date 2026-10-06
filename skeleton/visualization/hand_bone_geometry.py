"""Individual hand identity/placement proxies, never adopted osteometry.

The authored right hand uses a static spread/thumb pose. Articular surfaces,
motion, working opposition, tendons and manufacturable joints remain unresolved.
"""
from __future__ import annotations

import math

CARPAL_ROWS = {
    "SCAPHOID": "proximal", "LUNATE": "proximal",
    "TRIQUETRUM": "proximal", "PISIFORM": "proximal",
    "TRAPEZIUM": "distal", "TRAPEZOID": "distal",
    "CAPITATE": "distal", "HAMATE": "distal",
}
DIGIT_ROOTS = {"1": "TRAPEZIUM", "2": "TRAPEZOID", "3": "CAPITATE",
               "4": "HAMATE", "5": "HAMATE"}
POINTER = "PROJECTS/PHY_F28/profiles/f28.json#/hand_proxy_layout"


def validate_hand_layout(layout):
    """Reject missing identities, wrong chains/rows and invalid numeric inputs."""
    def vector(value, positive=False):
        return (isinstance(value, list) and len(value) == 3
                and all(type(v) in (int, float) and math.isfinite(v)
                        and (not positive or v > 0) for v in value))

    if (not isinstance(layout, dict) or layout.get("revision") != "HAND_V1"
            or not isinstance(layout.get("carpals"), dict)
            or set(layout["carpals"]) != set(CARPAL_ROWS)
            or not isinstance(layout.get("digits"), dict)
            or set(layout["digits"]) != set(DIGIT_ROOTS)):
        raise ValueError("invalid hand identity/layout contract")
    gap = layout.get("joint_gap_mm")
    if type(gap) not in (int, float) or not math.isfinite(gap) or gap <= 0:
        raise ValueError("invalid hand display gap")
    for code, row in CARPAL_ROWS.items():
        item = layout["carpals"][code]
        if (not isinstance(item, dict) or item.get("row") != row
                or not vector(item.get("center")) or not vector(item.get("radii_mm"), True)):
            raise ValueError("invalid carpal placement/section: " + code)
        if not 0 < item["center"][0] < 1 or not -1 < item["center"][1] < 1:
            raise ValueError("invalid normalized carpal placement")
    carpals = layout["carpals"]
    if max(v["center"][0] for v in carpals.values() if v["row"] == "proximal") >= min(
            v["center"][0] for v in carpals.values() if v["row"] == "distal"):
        raise ValueError("hand carpal rows are not proximal/distal ordered")
    for row in (("SCAPHOID", "LUNATE", "TRIQUETRUM"),
                ("TRAPEZIUM", "TRAPEZOID", "CAPITATE", "HAMATE")):
        if not all(carpals[a]["center"][1] > carpals[b]["center"][1] for a, b in zip(row, row[1:])):
            raise ValueError("hand carpal radial/ulnar order is inconsistent")
    if carpals["PISIFORM"]["center"][2] <= carpals["TRIQUETRUM"]["center"][2]:
        raise ValueError("pisiform must be palmar to triquetrum")
    for digit, root in DIGIT_ROOTS.items():
        item = layout["digits"][digit]
        count = 4 if digit == "1" else 5  # MC + two/three phalanges, plus end station.
        if (not isinstance(item, dict) or item.get("root") != root
                or not isinstance(item.get("stations"), list) or len(item["stations"]) != count
                or not all(vector(v) for v in item["stations"])
                or not isinstance(item.get("radii_mm"), list) or len(item["radii_mm"]) != count-1
                or not all(type(v) in (int, float) and math.isfinite(v) and v > 0 for v in item["radii_mm"])):
            raise ValueError("invalid hand digit chain/section: " + digit)
        if (not all(0 < v[0] <= 1 and -1 < v[1] < 1 for v in item["stations"])
                or not all(a[0] < b[0] for a, b in zip(item["stations"], item["stations"][1:]))):
            raise ValueError("invalid ordered hand digit stations")
    thumb, index = (layout["digits"][v]["stations"] for v in ("1", "2"))
    if thumb[-1][1] <= index[-1][1] or thumb[-1][2] <= index[-1][2]:
        raise ValueError("thumb review pose must be radial and palmar to index")


def build_hand(part, wrist, distal, radial, station, breadth, scale, layout):
    """Emit exactly 27 right-side identities; the existing core sink mirrors them."""
    from . import studio_model as sm
    from .bone_equivalence import expected_bones
    validate_hand_layout(layout)
    names = {r["bone_id"]: r["name"] for r in expected_bones()}
    anchors = {}
    palmar = [0, 1, 0]
    layout_hash = sm.digest(layout)

    def world(local):
        # Mesh coordinates are distal/palmar/radial: a right-handed basis.
        return [wrist[i]+distal[i]*local[0]+palmar[i]*local[1]+radial[i]*local[2]
                for i in range(3)]

    def point(values):
        # Profile coordinates: distal/hand-station, radial/hand-breadth, palmar mm.
        return world([values[0]*station, values[2]*scale, values[1]*breadth])

    def emit(code, mesh, key, **details):
        part(code.lower()+"_R", names["BONE_"+code+"_R"]+" / provisional hand proxy",
             "hands", "redwood", mesh, source=POINTER, role="bone_proxy",
             bone_id="BONE_"+code+"_R", geometry_inputs=[POINTER+"/"+key,
             POINTER+"/joint_gap_mm", "model#/design_datums/hand_station_mm",
             "model#/reference/statistics/handbreadth", "model#/landmarks/wrist_R"],
             hand_layout_sha256=layout_hash, dimensional_fidelity="unverified",
             physical_evidence="unmeasured", hand_frame_side="R",
             attachment_status="topology only; articular surfaces, joints, axes and tendons unresolved",
             **details)

    for code, item in layout["carpals"].items():
        center = point(item["center"])
        radii = item["radii_mm"]  # distal/radial/palmar mm, uniformly scaled.
        local = [item["center"][0]*station, item["center"][2]*scale, item["center"][1]*breadth]
        mesh = sm.ellipsoid(local, [radii[0]*scale, radii[2]*scale, radii[1]*scale])
        mesh["vertices"] = [world(v) for v in mesh["vertices"]]
        anchors["BONE_"+code+"_R"] = center
        emit(code, mesh, "carpals/"+code, center_mm=center, carpal_row=item["row"],
             topology_parent="BONE_TRIQUETRUM_R" if code == "PISIFORM" else None)

    gap = layout["joint_gap_mm"]*scale
    for digit, item in layout["digits"].items():
        points = [point(v) for v in item["stations"]]
        codes = ["META"+digit] + [f"PHAL_{digit}_{i}" for i in range(1, len(points)-1)]
        for i, (code, a, b, radius) in enumerate(zip(codes, points, points[1:], item["radii_mm"])):
            length = math.dist(a, b)
            if length <= gap:
                raise ValueError("hand display gap consumes a bone segment")
            axis = sm.mul(sm.sub(b, a), 1/length)
            endpoints = [sm.add(a, sm.mul(axis, gap/2)), sm.add(b, sm.mul(axis, -gap/2))]
            center = sm.mul(sm.add(a, b), .5)
            anchors["BONE_"+code+"_R"] = center
            parent = item["root"] if i == 0 else codes[i-1]
            emit(code, sm.tube(endpoints, radius*scale), "digits/"+digit,
                 digit_index=int(digit), chain_index=i, center_mm=center,
                 joint_stations_mm=[a, b], endpoints_mm=endpoints,
                 center_distance_mm=length-gap, display_gap_mm=gap,
                 topology_parent="BONE_"+parent+"_R",
                 topology_child="BONE_"+codes[i+1]+"_R" if i+1 < len(codes) else None)
    return anchors
