"""Final seven identity proxies; no measured anatomy or working mechanisms."""
from __future__ import annotations

import math

POINTER = "PROJECTS/PHY_F28/profiles/f28.json#/ear_hyoid_proxy_layout"
EAR_SOURCE = "https://openstax.org/books/anatomy-and-physiology-2e/pages/14-1-sensory-perception"
HYOID_SOURCE = "https://openstax.org/books/anatomy-and-physiology-2e/pages/7-2-the-skull"
CHAIN = ("MALLEUS", "INCUS", "STAPES")


def stirrup_mesh(points, sections):
    """Sweep a planar YZ loop with a fixed X normal, avoiding frame flips."""
    from . import studio_model as sm
    sides, vertices, faces = 12, [], []
    for i, point in enumerate(points):
        tangent = sm.unit(sm.sub(points[(i+1)%len(points)], points[i-1]))
        normal = [1, 0, 0]
        binormal = sm.cross(tangent, normal)
        radius = sections[i] if isinstance(sections, list) else sections
        for j in range(sides):
            theta = 2*math.pi*j/sides
            vertices.append(sm.add(point, sm.add(sm.mul(normal, radius*math.cos(theta)),
                                                sm.mul(binormal, radius*math.sin(theta)))))
    for i in range(len(points)):
        for j in range(sides):
            a, b = i*sides+j, i*sides+(j+1)%sides
            c, d = ((i+1)%len(points))*sides+(j+1)%sides, ((i+1)%len(points))*sides+j
            faces += [[a, b, c], [a, c, d]]
    return {"vertices": vertices, "faces": faces}


def validate_ear_hyoid_layout(layout):
    """Reject incomplete identities, invalid sections and reversed layouts."""
    def number(v):
        return type(v) in (int, float) and math.isfinite(v)

    def vector(v):
        return isinstance(v, list) and len(v) == 3 and all(number(x) for x in v)

    if (not isinstance(layout, dict)
            or set(layout) != {"revision", "authority", "units", "ear_origin_head_fraction",
                               "hyoid_origin_head_fraction", "ossicles", "hyoid"}
            or layout.get("revision") != "EAR_HYOID_V1"
            or any(not isinstance(layout.get(k), str) or not layout[k].strip() for k in ("authority", "units"))
            or not isinstance(layout.get("ossicles"), dict) or set(layout["ossicles"]) != set(CHAIN)):
        raise ValueError("invalid ear/hyoid identity/layout contract")
    ear, hyoid = layout["ear_origin_head_fraction"], layout["hyoid_origin_head_fraction"]
    if (not vector(ear) or not 0 < ear[0] < 1 or not -1 < ear[1] < 1 or not -1 < ear[2] < 0
            or not vector(hyoid) or hyoid[0] != 0 or not 0 < hyoid[1] < 1 or not -1.5 < hyoid[2] < -.9):
        raise ValueError("invalid ear or upper-neck display origin")
    for code, item in list(layout["ossicles"].items())+[("HYOID", layout["hyoid"])]:
        keys = {"centerline_mm", "radii_mm", "closed_centerline"}
        if code != "HYOID":
            keys.add("center_offset_mm")
        if (not isinstance(item, dict) or set(item) != keys
                or not isinstance(item.get("centerline_mm"), list)
                or not 3 <= len(item["centerline_mm"]) <= 80
                or not all(vector(v) for v in item["centerline_mm"])
                or item.get("closed_centerline") is not (code == "STAPES")
                or code != "HYOID" and not vector(item.get("center_offset_mm"))):
            raise ValueError("invalid ear/hyoid sweep: " + code)
        points = item["centerline_mm"]
        radius = item["radii_mm"]
        sections = radius if isinstance(radius, list) else [radius]*len(points)
        if (len(sections) != len(points) or not all(number(r) and r > 0 for r in sections)
                or any(math.dist(a, b) <= max(ra, rb) for a, b, ra, rb in
                       zip(points, points[1:]+points[:1] if code == "STAPES" else points[1:],
                           sections, sections[1:]+sections[:1] if code == "STAPES" else sections[1:]))
                or any(math.dist(a, c) < 1e-6 for a, c in zip(points, points[2:]))):
            raise ValueError("degenerate ear/hyoid sweep or consumed section")
    ossicles = layout["ossicles"]
    if not all(ossicles[a]["center_offset_mm"][0] > ossicles[b]["center_offset_mm"][0]
               for a, b in zip(CHAIN, CHAIN[1:])):
        raise ValueError("ear chain must run lateral malleus to medial stapes")
    ring = ossicles["STAPES"]["centerline_mm"]
    # A single convex planar loop preserves a stirrup opening without a guessed CAD union.
    if (any(v[0] != 0 for v in ring)
            or any((b[1]-a[1])*(c[2]-b[2])-(b[2]-a[2])*(c[1]-b[1]) >= -1e-8
                   for a, b, c in zip(ring, ring[1:]+ring[:1], ring[2:]+ring[:2]))):
        raise ValueError("stapes display loop must be clockwise and strictly convex in YZ")
    points = layout["hyoid"]["centerline_mm"]
    radius = layout["hyoid"]["radii_mm"]
    if (len(points) % 2 != 1 or points[0][0] <= 0 or points[-1][0] >= 0
            or any(not all(math.isclose(a, b, abs_tol=1e-9) for a, b in zip([-v[0], v[1], v[2]], w))
                   for v, w in zip(points, reversed(points)))
            or points[0][1] >= points[len(points)//2][1]
            or isinstance(radius, list) and radius != list(reversed(radius))):
        raise ValueError("hyoid must be one symmetric open U with posterior ends")


def build_ear_hyoid(part, center, radii, scale, layout, mandible_floor):
    """Emit three right ossicles and one hyoid; core reflects the left ossicles."""
    from . import studio_model as sm
    from .bone_equivalence import expected_bones
    validate_ear_hyoid_layout(layout)
    names = {r["bone_id"]: r["name"] for r in expected_bones()}
    layout_hash, anchors = sm.digest(layout), {}

    def origin(fractions):
        return sm.add(center, [v*r for v, r in zip(fractions, radii)])

    def emit(code, mesh, keys, **details):
        pid = code[:-2].lower()+"_R" if code.endswith("_R") else code.lower()
        part(pid, names["BONE_"+code]+" / provisional ear/hyoid proxy", "head", "redwood", mesh,
             source=POINTER, role="bone_proxy", bone_id="BONE_"+code,
             geometry_inputs=[POINTER+"/"+k for k in keys]+[
                 "model#/reference/statistics/headbreadth", "model#/reference/statistics/headlength",
                 "PROJECTS/PHY_F28/profiles/f28.json#/design_choices_mm/head_height",
                 "model#/height_mm", "model#/scale_from_mean"],
             ear_hyoid_layout_sha256=layout_hash,
             ear_hyoid_frame_side="R" if code.endswith("_R") else "midline",
             dimensional_fidelity="unverified", physical_evidence="unmeasured", motion_implemented=False,
             attachment_status="partial topology only; surfaces, joint interfaces and soft-tissue mechanisms unresolved",
             **details)

    ear_origin = origin(layout["ear_origin_head_fraction"])
    for i, code in enumerate(CHAIN):
        item = layout["ossicles"][code]
        anchor = sm.add(ear_origin, sm.mul(item["center_offset_mm"], scale))
        points = [sm.add(anchor, sm.mul(v, scale)) for v in item["centerline_mm"]]
        sections = sm.mul(item["radii_mm"], scale) if isinstance(item["radii_mm"], list) else item["radii_mm"]*scale
        rs = sections if isinstance(sections, list) else [sections]*len(points)
        if (any(v[0] <= r for v, r in zip(points, rs))
                or any(abs(v[j]-center[j])+r >= radii[j] for v, r in zip(points, rs) for j in range(3))):
            raise ValueError("ossicle section crosses midline or leaves the head display frame")
        parent = "BONE_"+CHAIN[i-1]+"_R" if i else None
        child = "BONE_"+CHAIN[i+1]+"_R" if i < 2 else None
        neighbors = [v for v in (parent, child) if v]
        anchors["BONE_"+code+"_R"] = anchor
        mesh = stirrup_mesh(points, sections) if code == "STAPES" else sm.tube(points, sections)
        emit(code+"_R", mesh,
             ["ossicles/"+code, "ear_origin_head_fraction"], center_mm=anchor,
             centerline_mm=points, closed_centerline=item["closed_centerline"],
             proxy_shape="stirrup_loop" if code == "STAPES" else "tapered_sweep",
             chain_index=i, housing_bone_id="BONE_TEMP_R", topology_parent=parent,
             topology_child=child, topology_neighbors=neighbors, articulates_with=neighbors,
             topology_source=EAR_SOURCE,
             non_bone_connection="tympanic_membrane" if i == 0 else "oval_window" if i == 2 else None)
    item = layout["hyoid"]
    hyoid_origin = origin(layout["hyoid_origin_head_fraction"])
    points = [sm.add(hyoid_origin, sm.mul(v, scale)) for v in item["centerline_mm"]]
    sections = sm.mul(item["radii_mm"], scale) if isinstance(item["radii_mm"], list) else item["radii_mm"]*scale
    mesh = sm.tube(points, sections)
    if max(v[2] for v in mesh["vertices"]) >= mandible_floor:
        raise ValueError("hyoid display must remain below the mandible")
    anchor = points[len(points)//2]
    anchors["BONE_HYOID"] = anchor
    emit("HYOID", mesh, ["hyoid", "hyoid_origin_head_fraction"], center_mm=anchor,
         centerline_mm=points, closed_centerline=False, proxy_shape="open_U_sweep",
         topology_parent=None, topology_child=None, topology_neighbors=[], articulates_with=[],
         topology_source=HYOID_SOURCE, non_bone_support="muscle_and_ligament_suspension_unmodeled")
    return anchors
