"""Individual skull display proxies, with no adopted bone measurements.

Shell patches, ellipsoids and sweeps describe identity and coarse distribution.
They do not reproduce sutures, foramina, sinuses, dentition or articular surfaces.
"""
from __future__ import annotations

import math

POINTER = "PROJECTS/PHY_F28/profiles/f28.json#/skull_proxy_layout"
TOPOLOGY_SOURCE = "https://openstax.org/books/anatomy-and-physiology-2e/pages/7-2-the-skull"
SHELL_IDS = {"FRONTAL", "PAR_R", "OCCIPITAL", "TEMP_R"}
ELLIPSOID_IDS = {"SPHENOID", "ETHMOID", "MAXILLA_R", "NASAL_R", "LACRIMAL_R", "PALATINE_R", "VOMER"}
SWEEP_IDS = {"ZYGOMATIC_R", "INFERIOR_NASAL_CONCHA_R", "MANDIBLE"}

# Selected reciprocal adjacencies only; not a complete suture/contact graph.
NEIGHBORS = {
    "FRONTAL": ["PAR_R", "PAR_L", "NASAL_R", "NASAL_L", "LACRIMAL_R", "LACRIMAL_L"],
    "PAR_R": ["FRONTAL", "PAR_L", "OCCIPITAL", "TEMP_R"],
    "OCCIPITAL": ["PAR_R", "PAR_L", "TEMP_R", "TEMP_L"],
    "TEMP_R": ["PAR_R", "OCCIPITAL", "SPHENOID", "ZYGOMATIC_R", "MANDIBLE"],
    "SPHENOID": ["TEMP_R", "TEMP_L", "ETHMOID"],
    "ETHMOID": ["SPHENOID", "VOMER", "LACRIMAL_R", "LACRIMAL_L",
                "INFERIOR_NASAL_CONCHA_R", "INFERIOR_NASAL_CONCHA_L"],
    "MAXILLA_R": ["MAXILLA_L", "NASAL_R", "ZYGOMATIC_R", "LACRIMAL_R", "PALATINE_R",
                  "INFERIOR_NASAL_CONCHA_R", "VOMER"],
    "ZYGOMATIC_R": ["TEMP_R", "MAXILLA_R"],
    "NASAL_R": ["FRONTAL", "NASAL_L", "MAXILLA_R"],
    "LACRIMAL_R": ["FRONTAL", "ETHMOID", "MAXILLA_R"],
    "PALATINE_R": ["PALATINE_L", "MAXILLA_R", "INFERIOR_NASAL_CONCHA_R", "VOMER"],
    "INFERIOR_NASAL_CONCHA_R": ["ETHMOID", "MAXILLA_R", "PALATINE_R"],
    "VOMER": ["ETHMOID", "MAXILLA_R", "MAXILLA_L", "PALATINE_R", "PALATINE_L"],
    "MANDIBLE": ["TEMP_R", "TEMP_L"],
}


def validate_skull_layout(layout):
    """Reject malformed, degenerate or reversed provisional skull placements."""
    def number(value, positive=False):
        return type(value) in (int, float) and math.isfinite(value) and (not positive or value > 0)

    def vector(value, positive=False):
        return (isinstance(value, list) and len(value) == 3
                and all(number(v, positive) and abs(v) <= 1 for v in value))

    if (not isinstance(layout, dict)
            or set(layout) != {"revision", "authority", "units", "shell_inset_mm", "shell_thickness_mm",
                               "shell_samples", "shells", "ellipsoids", "sweeps"}
            or layout.get("revision") != "SKULL_V1"
            or any(not isinstance(layout.get(k), str) or not layout[k].strip() for k in ("authority", "units"))
            or any(not isinstance(layout.get(k), dict) or set(layout[k]) != wanted
                   for k, wanted in (("shells", SHELL_IDS), ("ellipsoids", ELLIPSOID_IDS), ("sweeps", SWEEP_IDS)))):
        raise ValueError("invalid skull identity/layout contract")
    for key in ("shell_inset_mm", "shell_thickness_mm"):
        if not number(layout.get(key), True):
            raise ValueError("invalid skull shell section")
    samples = layout.get("shell_samples")
    if (not isinstance(samples, list) or len(samples) != 2
            or any(type(v) is not int or not 4 <= v <= 40 for v in samples)):
        raise ValueError("invalid skull shell sampling")
    for code, item in layout["shells"].items():
        if not isinstance(item, dict) or set(item) != {"azimuth_deg", "polar_deg"}:
            raise ValueError("invalid skull shell: " + code)
        theta, phi = item.get("azimuth_deg"), item.get("polar_deg")
        if (any(not isinstance(v, list) or len(v) != 2 or not all(number(n) for n in v) for v in (theta, phi))
                or not -180 <= theta[0] < theta[1] <= 360 or theta[1]-theta[0] >= 180
                or not 5 <= phi[0] < phi[1] <= 175):
            raise ValueError("invalid skull shell angles: " + code)
        if code.endswith("_R") and not -90 < theta[0] < theta[1] < 90:
            raise ValueError("paired skull shell must stay on the right")
        if code == "FRONTAL" and not math.isclose(sum(theta), 180):
            raise ValueError("frontal shell must be symmetric and anterior")
        if code == "OCCIPITAL" and not math.isclose(sum(theta), 540):
            raise ValueError("occipital shell must be symmetric and posterior")
    if layout["shells"]["PAR_R"]["polar_deg"][1] >= layout["shells"]["TEMP_R"]["polar_deg"][0]:
        raise ValueError("parietal must lie above the temporal display patch")
    for code, item in layout["ellipsoids"].items():
        if (not isinstance(item, dict) or set(item) != {"center", "radii"}
                or not vector(item.get("center")) or not vector(item.get("radii"), True)
                or any(abs(c)+r > 1 for c, r in zip(item["center"], item["radii"]))):
            raise ValueError("invalid skull center/section: " + code)
        if code.endswith("_R"):
            if item["center"][0] <= item["radii"][0]:
                raise ValueError("paired skull proxy must stay on the right")
        elif item["center"][0] != 0:
            raise ValueError("unpaired skull proxy must be centered")
    for code, item in layout["sweeps"].items():
        if (not isinstance(item, dict) or set(item) != {"centerline", "radius_mm"}
                or not isinstance(item.get("centerline"), list)
                or not 3 <= len(item["centerline"]) <= 80
                or not all(vector(v) for v in item["centerline"])
                or not number(item.get("radius_mm"), True)):
            raise ValueError("invalid skull sweep: " + code)
        points = item["centerline"]
        if (any(math.dist(a, b) < 1e-6 for a, b in zip(points, points[1:]))
                or any(math.dist(a, c) < 1e-6 for a, c in zip(points, points[2:]))
                or (code.endswith("_R") and any(v[0] <= 0 for v in points))):
            raise ValueError("degenerate or wrong-side skull sweep")
    e = layout["ellipsoids"]
    if (e["ETHMOID"]["center"][2] <= e["VOMER"]["center"][2]
            or e["NASAL_R"]["center"][2] <= e["MAXILLA_R"]["center"][2]
            or e["PALATINE_R"]["center"][1] >= e["MAXILLA_R"]["center"][1]
            or e["LACRIMAL_R"]["center"][0] >= layout["sweeps"]["ZYGOMATIC_R"]["centerline"][0][0]):
        raise ValueError("skull septum/face display ordering is inconsistent")
    jaw = layout["sweeps"]["MANDIBLE"]["centerline"]
    if (len(jaw) % 2 != 1 or jaw[0][0] <= 0 or jaw[-1][0] >= 0
            or any(not all(math.isclose(a, b, abs_tol=1e-9) for a, b in zip([-v[0], v[1], v[2]], w))
                   for v, w in zip(jaw, reversed(jaw)))
            or jaw[0][2] <= jaw[len(jaw)//2][2] or jaw[0][1] >= jaw[len(jaw)//2][1]
            or jaw[len(jaw)//2][2] >= e["MAXILLA_R"]["center"][2]):
        raise ValueError("mandible must be one symmetric lower U with superior posterior ends")


def shell_patch(center, radii, azimuth, polar, thickness, samples):
    """Closed ellipsoidal shell sector with outward faces and capped borders."""
    inner = [r-thickness for r in radii]
    if min(inner) <= 0:
        raise ValueError("skull shell thickness consumes the head envelope")
    columns, rows = samples
    vertices, faces = [], []
    for axes in (radii, inner):
        for i in range(rows+1):
            phi = math.radians(polar[0]+(polar[1]-polar[0])*i/rows)
            for j in range(columns+1):
                theta = math.radians(azimuth[0]+(azimuth[1]-azimuth[0])*j/columns)
                vertices.append([center[0]+axes[0]*math.sin(phi)*math.cos(theta),
                                 center[1]+axes[1]*math.sin(phi)*math.sin(theta),
                                 center[2]+axes[2]*math.cos(phi)])
    n = (rows+1)*(columns+1)
    for i in range(rows):
        for j in range(columns):
            a = i*(columns+1)+j
            b, c, d = a+columns+1, a+columns+2, a+1
            faces += [[a, b, c], [a, c, d], [n+a, n+c, n+b], [n+a, n+d, n+c]]
    # Counter-clockwise outer border as seen from outside; close toward inner.
    border = [j for j in range(columns+1)]
    border += [i*(columns+1)+columns for i in range(1, rows+1)]
    border += [rows*(columns+1)+j for j in range(columns-1, -1, -1)]
    border += [i*(columns+1) for i in range(rows-1, 0, -1)]
    for a, b in zip(border, border[1:]+border[:1]):
        faces += [[a, b, n+b], [a, n+b, n+a]]
    return {"vertices": vertices, "faces": faces}


def build_skull(part, center, radii, scale, layout):
    """Emit six midline and eight right-side identities; core reflects the eight."""
    from . import studio_model as sm
    from .bone_equivalence import expected_bones
    validate_skull_layout(layout)
    names = {r["bone_id"]: r["name"] for r in expected_bones()}
    layout_hash, anchors = sm.digest(layout), {}

    def point(values):
        return sm.add(center, [v*r for v, r in zip(values, radii)])

    def emit(code, mesh, key, **details):
        pid = code[:-2].lower()+"_R" if code.endswith("_R") else code.lower()
        part(pid, names["BONE_"+code]+" / provisional skull proxy", "head", "redwood", mesh,
             source=POINTER, role="bone_proxy", bone_id="BONE_"+code,
             geometry_inputs=[POINTER+"/"+key, "model#/reference/statistics/headbreadth",
                              "model#/reference/statistics/headlength",
                              "PROJECTS/PHY_F28/profiles/f28.json#/design_choices_mm/head_height",
                              "model#/height_mm", "model#/scale_from_mean"]
                             + ([POINTER+"/shell_inset_mm", POINTER+"/shell_thickness_mm", POINTER+"/shell_samples"]
                                if key.startswith("shells/") else []),
             skull_layout_sha256=layout_hash, skull_frame_side="R" if code.endswith("_R") else "midline",
             dimensional_fidelity="unverified", physical_evidence="unmeasured",
             topology_neighbors=["BONE_"+v for v in NEIGHBORS[code]], topology_source=TOPOLOGY_SOURCE,
             attachment_status="selected adjacency only; sutures, articular surfaces and joint interfaces unresolved",
             **details)

    for code, item in layout["shells"].items():
        axes = [r-layout["shell_inset_mm"]*scale for r in radii]
        mesh = shell_patch(center, axes, item["azimuth_deg"], item["polar_deg"],
                           layout["shell_thickness_mm"]*scale, layout["shell_samples"])
        phi, theta = (math.radians(sum(item[k])/2) for k in ("polar_deg", "azimuth_deg"))
        direction = [math.sin(phi)*math.cos(theta), math.sin(phi)*math.sin(theta), math.cos(phi)]
        anchor = sm.add(center, [v*(r-layout["shell_thickness_mm"]*scale/2) for v, r in zip(direction, axes)])
        anchors["BONE_"+code] = anchor
        emit(code, mesh, "shells/"+code, center_mm=anchor, proxy_shape="shell_patch")
    for code, item in layout["ellipsoids"].items():
        anchor = point(item["center"])
        anchors["BONE_"+code] = anchor
        emit(code, sm.ellipsoid(anchor, [v*r for v, r in zip(item["radii"], radii)]),
             "ellipsoids/"+code, center_mm=anchor, proxy_shape="ellipsoid")
    for code, item in layout["sweeps"].items():
        points = [point(v) for v in item["centerline"]]
        radius = item["radius_mm"]*scale
        if (any(math.dist(a, b) <= 2*radius for a, b in zip(points, points[1:]))
                or code.endswith("_R") and min(v[0] for v in points) <= radius):
            raise ValueError("skull sweep section consumes a station or crosses midline")
        anchor = points[len(points)//2]
        anchors["BONE_"+code] = anchor
        emit(code, sm.tube(points, radius), "sweeps/"+code,
             center_mm=anchor, centerline_mm=points, closed_centerline=False, proxy_shape="open_sweep",
             **({"articulates_with": ["BONE_TEMP_R", "BONE_TEMP_L"],
                 "joint_stations_mm": [points[0], points[-1]], "motion_implemented": False}
                if code == "MANDIBLE" else {}))
    return anchors
