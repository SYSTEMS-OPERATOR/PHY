"""Bounded 71-bone distribution overlay. Every dimension is provisional.

Count/adjacency is anatomically grounded; mesh shape and placement are not
osteometry. These meshes cannot update the canonical BoneSpec inventory.
"""
from __future__ import annotations

import copy
import math

from .bone_equivalence import bone_audit, expected_bones
from .hand_bone_geometry import build_hand
from .foot_bone_geometry import build_foot
from .skull_bone_geometry import build_skull


def validate_layout(layout):
    def numeric(value):
        if isinstance(value, dict):
            for item in value.values():
                numeric(item)
        elif isinstance(value, list):
            for item in value:
                numeric(item)
        elif type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError("invalid bone proxy parameter")
    for key, value in layout.items():
        if key not in ("revision", "authority", "units"):
            numeric(value)
    for key, size in (("spine_y_mm", 8), ("rib_breadth_fractions", 12),
                      ("rib_depth_fractions", 12), ("rib_drop_mm", 12),
                      ("scapula_vertices_relative_shoulder_mm", 3),
                      ("hip_outline_offsets_mm", 8), ("hip_outline_breadth_fractions", 3),
                      ("forearm_separation_mm", 2), ("patella_offset_mm", 3)):
        if len(layout[key]) != size:
            raise ValueError("invalid bone proxy parameter size: " + key)
    positive_keys = [k for k in layout if k.endswith(("_radii_mm", "_radius_mm", "_thickness_mm"))]
    positive_keys += ["forearm_separation_mm", "hip_outline_offsets_mm", "rib_drop_mm",
                      "c1_below_vertex_mm", "t1_below_cervicale_mm", "l1_above_tenth_rib_mm",
                      "t12_above_l1_mm", "l5_above_hip_mm", "coccyx_below_hip_mm",
                      "rib_posterior_offset_mm", "sternum_half_width_mm"]
    def positive(value):
        if isinstance(value, dict):
            return all(positive(v) for v in value.values())
        if isinstance(value, list):
            return bool(value) and all(positive(v) for v in value)
        return type(value) in (int, float) and math.isfinite(value) and value > 0
    if any(not positive(layout[k]) for k in positive_keys):
        raise ValueError("bone proxy sizes must be positive")
    if set(layout["vertebra_radii_mm"]) != {"C", "T", "L"} or any(len(v) != 3 for v in layout["vertebra_radii_mm"].values()):
        raise ValueError("invalid proxy radius vector")
    for key, size in (("sacrum_radii_mm", 3), ("coccyx_radii_mm", 3),
                      ("radius_radii_mm", 2), ("ulna_radii_mm", 2), ("patella_radii_mm", 3)):
        if len(layout[key]) != size:
            raise ValueError("invalid proxy radius vector")
    if any(len(v) != 3 for v in layout["scapula_vertices_relative_shoulder_mm"]):
        raise ValueError("invalid scapula vertex")
    for key in ("rib_breadth_fractions", "rib_depth_fractions", "hip_outline_breadth_fractions"):
        if any(type(v) not in (int, float) or not 0 < v <= 1 for v in layout[key]):
            raise ValueError("invalid internal breadth/depth fraction")
    if type(layout["rib_sweep_samples"]) is not int or not 12 <= layout["rib_sweep_samples"] <= 100:
        raise ValueError("invalid rib sampling")
    if not 0 < layout["rib_free_end_angle_deg"] < 90:
        raise ValueError("invalid floating rib endpoint")


def plate_xz(outline, y, half_thickness):
    """Convex plate only; fail on a non-convex/degenerate input instead of guessing."""
    values = [y, half_thickness] + [v for point in outline for v in point]
    if len(outline) < 3 or any(len(point) != 2 for point in outline) or not half_thickness > 0 or any(not math.isfinite(v) for v in values):
        raise ValueError("invalid proxy plate dimensions")
    area = sum(a[0]*b[1]-b[0]*a[1] for a, b in zip(outline, outline[1:]+outline[:1]))
    if area < 0:
        outline = list(reversed(outline))
    turns = [(b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0])
             for a, b, c in zip(outline, outline[1:]+outline[:1], outline[2:]+outline[:2])]
    if any(v <= 1e-8 for v in turns):
        raise ValueError("proxy plate outline must be strictly convex")
    n = len(outline)
    vertices = [[x, y+t, z] for t in (-half_thickness, half_thickness) for x, z in outline]
    faces = []
    for i in range(1, n-1):
        faces += [[0, i, i+1], [n, n+i+1, n+i]]
    for i in range(n):
        j = (i+1) % n
        faces += [[i, n+j, j], [i, n+i, n+j]]
    return {"vertices": vertices, "faces": faces}


def build_core(parts, part, model_inputs, layout, hand_layout, foot_layout, skull_layout, head_frame, landmarks, hand_station):
    """Add replacements to the existing part sink, then classify and mirror it."""
    from . import studio_model as sm
    validate_layout(layout)
    s, height, m, shoulder_x, shoulder_z, hip_x, hip_z, pelvis = model_inputs
    p = layout
    names = {r["bone_id"]: r["name"] for r in expected_bones()}
    pointer = "PROJECTS/PHY_F28/profiles/f28.json#/bone_proxy_layout"

    def emit(pid, code, region, mesh, keys, **details):
        part(pid, names["BONE_"+code]+" / provisional proxy", region, "redwood", mesh,
             source=pointer, role="bone_proxy", bone_id="BONE_"+code,
             geometry_inputs=[pointer+"/"+key for key in keys],
             dimensional_fidelity="unverified", physical_evidence="unmeasured", **details)

    anchors = {}
    c1, c7 = height-p["c1_below_vertex_mm"]*s, m["cervicaleheight"]
    t1 = c7-p["t1_below_cervicale_mm"]*s
    l1, l5 = m["tenthribheight"]+p["l1_above_tenth_rib_mm"]*s, hip_z+p["l5_above_hip_mm"]*s
    t12 = l1+p["t12_above_l1_mm"]*s
    if not c1 > c7 > t1 > t12 > l1 > l5 > hip_z:
        raise ValueError("spine station ordering is inconsistent")
    ys = p["spine_y_mm"]
    for prefix, count, ztop, zbottom, ya, yb in (("C", 7, c1, c7, ys[0], ys[1]),
                                               ("T", 12, t1, t12, ys[1], ys[3]),
                                               ("L", 5, l1, l5, ys[4], ys[5])):
        for i in range(count):
            t = i/(count-1)
            y = ya+(yb-ya)*t
            if prefix == "T":
                y += (ys[2]-(ya+yb)/2)*math.sin(math.pi*t)
            code = f"{prefix}{i+1}"
            center = [0, y*s, ztop+(zbottom-ztop)*t]
            anchors["BONE_"+code] = center
            emit("vertebra_"+code, code, "spine", sm.ellipsoid(center, sm.mul(p["vertebra_radii_mm"][prefix], s)),
                 ["spine_y_mm", "vertebra_radii_mm", "c1_below_vertex_mm", "t1_below_cervicale_mm",
                  "t12_above_l1_mm", "l1_above_tenth_rib_mm", "l5_above_hip_mm"], center_mm=center)
    for code, z, y, radii, keys in (
        ("SACRUM", hip_z+p["sacrum_offset_from_hip_mm"]*s, ys[6], p["sacrum_radii_mm"], ["sacrum_offset_from_hip_mm", "sacrum_radii_mm"]),
        ("COCCYX", hip_z-p["coccyx_below_hip_mm"]*s, ys[7], p["coccyx_radii_mm"], ["coccyx_below_hip_mm", "coccyx_radii_mm"])):
        center = [0, y*s, z]; anchors["BONE_"+code] = center
        emit(code.lower(), code, "spine", sm.ellipsoid(center, sm.mul(radii, s)), keys+["spine_y_mm"], center_mm=center)

    sternum_top = [0, m["chestdepth"]*p["rib_depth_fractions"][1]/2, anchors["BONE_T2"][2]-p["rib_drop_mm"][1]*s]
    sternum_bottom = [0, m["chestdepth"]*p["rib_depth_fractions"][6]/2, anchors["BONE_T7"][2]-p["rib_drop_mm"][6]*s]
    emit("sternum", "STERNUM", "thorax", sm.tube([sternum_top, sternum_bottom], p["sternum_radius_mm"]*s),
         ["sternum_radius_mm", "rib_depth_fractions", "rib_drop_mm"], endpoints_mm=[sternum_top, sternum_bottom])
    for level in range(1, 13):
        anchor = anchors[f"BONE_T{level}"]
        start = sm.add(anchor, [p["rib_posterior_offset_mm"]*s, 0, 0])
        rx, ry = m["chestbreadth"]*p["rib_breadth_fractions"][level-1]/2, m["chestdepth"]*p["rib_depth_fractions"][level-1]/2
        end_angle = math.radians(p["rib_free_end_angle_deg"]) if level >= 11 else math.pi/2
        points = []
        for i in range(p["rib_sweep_samples"]):
            t = i/(p["rib_sweep_samples"]-1)
            theta = -math.pi/2 + (end_angle+math.pi/2)*t
            # Starts at its T-level datum; ends open, never a complete ring.
            x = rx*math.cos(theta)+start[0]*(1-t)
            if level <= 10:
                x += p["sternum_half_width_mm"]*s*t
            y = ry*math.sin(theta)+(start[1]+ry)*(1-t)
            points.append([x, y, start[2]-p["rib_drop_mm"][level-1]*s*t])
        points[0] = start
        anterior = "via_costal_cartilage_to_sternum" if level <= 7 else "via_costal_margin" if level <= 10 else "free"
        emit(f"rib_{level}_R", f"RIB{level}_R", "thorax", sm.tube(points, p["rib_radius_mm"]*s),
             ["rib_breadth_fractions", "rib_depth_fractions", "rib_drop_mm", "rib_radius_mm",
              "rib_posterior_offset_mm", "rib_sweep_samples", "rib_free_end_angle_deg", "sternum_half_width_mm"],
             centerline_mm=points, posterior_attachment=f"BONE_T{level}", anterior_attachment=anterior,
             attachment_status="topology only; cartilage and joint interfaces are not modeled", closed_centerline=False)

    scapula = [[shoulder_x+x*s, y*s, shoulder_z+z*s] for x, y, z in p["scapula_vertices_relative_shoulder_mm"]]
    # A planar plate is a topology proxy, not a reproduced scapular surface.
    scap_y = sum(v[1] for v in scapula)/3
    emit("scapula_R", "SCAPULA_R", "shoulders",
         plate_xz([[v[0], v[2]] for v in scapula], scap_y, p["scapula_thickness_mm"]*s/2),
         ["scapula_vertices_relative_shoulder_mm", "scapula_thickness_mm"],
         attachment_status="unresolved glenoid and scapulothoracic interfaces")
    o = [v*s for v in p["hip_outline_offsets_mm"]]; f = p["hip_outline_breadth_fractions"]
    outline = [[o[0], hip_z-o[1]], [hip_x+o[2], hip_z-o[3]],
               [pelvis*f[0], hip_z+o[4]], [pelvis*f[1], m["iliocristaleheight"]+o[5]],
               [pelvis*f[2], m["iliocristaleheight"]+o[6]], [o[0], hip_z+o[7]]]
    emit("hip_bone_R", "HIP_R", "pelvis", plate_xz(outline, 0, p["hip_plate_half_thickness_mm"]*s),
         ["hip_outline_offsets_mm", "hip_outline_breadth_fractions", "hip_plate_half_thickness_mm"],
         attachment_status="one adult fused hip proxy; acetabulum and SI interfaces unresolved")

    by_id = {v["id"]: v for v in parts}
    elbow, wrist = landmarks["elbow_R"], landmarks["wrist_R"]
    direction = sm.unit(sm.sub(wrist, elbow))
    lateral = [ -direction[2], 0, direction[0] ]  # +x in neutral A/T poses = thumb/radial side.
    for code, sign, radii in (("RADIUS", 1, p["radius_radii_mm"]), ("ULNA", -1, p["ulna_radii_mm"])):
        endpoints = [sm.add(point, sm.mul(lateral, sign*sep*s/2))
                     for point, sep in zip((elbow, wrist), p["forearm_separation_mm"])]
        emit(code.lower()+"_R", code+"_R", "arms", sm.tube(endpoints, sm.mul(radii, s)),
             ["forearm_separation_mm", code.lower()+"_radii_mm"], endpoints_mm=endpoints,
             center_distance_mm=math.dist(*endpoints), lateral_side="radial" if sign == 1 else "ulnar")
    knee = by_id["femur_R"]["endpoints_mm"][1]
    center = sm.add(knee, sm.mul(p["patella_offset_mm"], s))
    emit("patella_R", "PATELLA_R", "legs", sm.ellipsoid(center, sm.mul(p["patella_radii_mm"], s)),
         ["patella_offset_mm", "patella_radii_mm"], center_mm=center)
    anchors.update(build_hand(part, wrist, direction, lateral, hand_station, m["handbreadth"], s, hand_layout))
    anchors.update(build_foot(part, landmarks["ankle_R"], landmarks["floor"],
                              m["footlength"], m["footbreadthhorizontal"], s, foot_layout))
    anchors.update(build_skull(part, *head_frame, s, skull_layout))

    # Retained long-bone/clavicle shapes remain external-station proxies.
    mapping = {"clavicle": "CLAVICLE", "upper_arm": "HUMERUS", "femur": "FEMUR", "tibia": "TIBIA", "fibular": "FIBULA"}
    for item in parts:
        pid = item["id"]
        if item.get("role") == "bone_proxy":
            continue
        prefix, _, side = pid.rpartition("_")
        if prefix in mapping and side in ("R", "L"):
            item.update(role="bone_proxy", bone_id="BONE_"+mapping[prefix]+"_"+side,
                        dimensional_fidelity="unverified", physical_evidence="unmeasured",
                        geometry_inputs=["PROJECTS/PHY_F28/profiles/f28.json#/design_choices_mm",
                                         "model#/design_datums", "model#/reference/statistics"])
        elif item["region"] == "envelope":
            item["role"] = "envelope"
        elif "_joint_" in pid:
            item["role"] = "hardware"
        elif pid.startswith(("shoulder_bridge", "chest_form_", "hip_form_")):
            item["role"] = "support"
        else:
            item["role"] = "form"

    # Author right-side bones once; reflect vertices AND winding/attachment data.
    right = [v for v in parts if v.get("bone_id", "").endswith("_R")]
    left_ids = {v["id"] for v in parts}
    for item in right:
        other = copy.deepcopy(item)
        other["id"] = item["id"][:-1]+"L"
        other["bone_id"] = item["bone_id"][:-1]+"L"
        other["name"] = item["name"].replace("R ", "L ").replace("_R", "_L")
        other["vertices"] = [[-v[0], v[1], v[2]] for v in item["vertices"]]
        other["faces"] = [[a, c, b] for a, b, c in item["faces"]]
        other["bounds_mm"] = {"min": [-item["bounds_mm"]["max"][0], *item["bounds_mm"]["min"][1:]],
                              "max": [-item["bounds_mm"]["min"][0], *item["bounds_mm"]["max"][1:]]}
        for key in ("endpoints_mm", "centerline_mm", "joint_stations_mm"):
            if key in item:
                other[key] = [[-v[0], v[1], v[2]] for v in item[key]]
        if "center_mm" in item:
            v = item["center_mm"]; other["center_mm"] = [-v[0], v[1], v[2]]
        for key in ("topology_parent", "topology_child"):
            if item.get(key, "") and item[key].endswith("_R"):
                other[key] = item[key][:-1]+"L"
        for key in ("topology_neighbors", "articulates_with"):
            if key in item:
                other[key] = [v[:-1]+("L" if v.endswith("_R") else "R")
                              if v.endswith(("_R", "_L")) else v for v in item[key]]
        if "skull_frame_side" in item:
            other["skull_frame_side"] = "L"
        if "hand_frame_side" in item:
            other["hand_frame_side"] = "L"
            other["geometry_inputs"] = [v.replace("wrist_R", "wrist_L") for v in item["geometry_inputs"]]
        if "foot_frame_side" in item:
            other["foot_frame_side"] = "L"
            other["geometry_inputs"] = [v.replace("ankle_R", "ankle_L") for v in item["geometry_inputs"]]
        other["mirror_of"] = item["id"]
        if other["id"] in left_ids:
            index = next(i for i, v in enumerate(parts) if v["id"] == other["id"])
            parts[index] = other
        else:
            parts.append(other)
    audit = bone_audit(parts)
    if any(audit[k] for k in ("missing_core_bones", "missing_hand_bones", "missing_foot_bones", "missing_skull_bones")):
        raise ValueError("core/hand/foot/skull bone coverage is incomplete")
    skull_parts = {p["bone_id"]: p for p in parts if p.get("skull_layout_sha256")}
    for row in audit["bones"]:
        if row["bone_id"] in skull_parts:
            skull_parts[row["bone_id"]].update(source_record=row["source_record"],
                                              source_record_status=row["source_record_status"])
    for key in list(anchors):
        if key.endswith("_R"):
            v = anchors[key]; anchors[key[:-1]+"L"] = [-v[0], v[1], v[2]]
    return audit, anchors
