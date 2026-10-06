"""Deterministic reference armature geometry for PHY Studio.

These are mechanical form-study parts, not new canonical BoneSpec records.
External anthropometry, proposed joint datums, and fabrication qualification
are deliberately distinct. No function changes SOPHY canon or A0 design data.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import statistics

from .core_bone_geometry import build_core

ROOT = Path(__file__).resolve().parents[2]
PROJECT = ROOT / "PROJECTS/PHY_F28"


def read(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def reference_statistics():
    source = read(PROJECT / "references/ansur_age28.json")
    provenance = read(PROJECT / "references/provenance.json")
    catalog = read(ROOT / provenance["source_catalog"])
    if digest(source) != provenance["snapshot_sha256"]:
        raise ValueError("F28 projected source hash mismatch")
    if source["source_csv_sha256"] != catalog["ansur"]["csv_sha256"]:
        raise ValueError("F28 source differs from pinned ANSUR evidence")
    columns, rows = source["columns"], source["rows"]
    if source["units"] != "mm" or source["age_years"] != 28 or len(rows) != 92 or len(set(columns)) != len(columns):
        raise ValueError("invalid F28 population, units or columns")
    if len(set(row[0] for row in rows)) != len(rows):
        raise ValueError("duplicate source rows")
    for row in rows:
        if len(row) != len(columns) or any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0 for v in row):
            raise ValueError("invalid reference row")
    # Every previously pinned field must agree with the independent full-population projection.
    old = read(ROOT / "PROJECTS/T56_CARBON/references/ansur_female_dimensions.json")
    old_rows = {r[0]: dict(zip(old["columns"], r)) for r in old["rows"] if r[1] == 28}
    if set(old_rows) != {r[0] for r in rows}:
        raise ValueError("age-28 cohort differs from the existing projection")
    stats = {}
    for i, key in enumerate(columns[1:], 1):
        values = [row[i] for row in rows]
        if key in old["columns"] and any(row[i] != old_rows[row[0]][key] for row in rows):
            raise ValueError("projection disagreement: " + key)
        stats[key] = {"mean_mm": round(statistics.mean(values), 6),
                      "sd_mm": round(statistics.stdev(values), 6), "n": len(values)}
    return {"source": source["source_id"], "provenance": provenance,
            "population": source["population"], "age_years": 28, "n": len(rows),
            "snapshot_sha256": provenance["snapshot_sha256"], "statistics": stats,
            "data_url": catalog["ansur"]["data_url"], "report_url": catalog["ansur"]["report_url"]}


def add(a, b):
    return [a[i] + b[i] for i in range(3)]


def sub(a, b):
    return [a[i] - b[i] for i in range(3)]


def mul(a, f):
    return [v * f for v in a]


def cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


def unit(a):
    length = math.sqrt(sum(v*v for v in a))
    if length < 1e-10:
        raise ValueError("zero direction")
    return mul(a, 1/length)


def tube(points, radius, closed=False, sides=12):
    """Watertight polygonal sweep used identically by viewer and mesh exports."""
    vertices, faces = [], []
    count = len(points)
    for i, point in enumerate(points):
        prev = points[(i-1) % count] if closed or i else point
        nex = points[(i+1) % count] if closed or i < count-1 else point
        t = unit(sub(nex, prev))
        n = unit(cross(t, [0, 0, 1] if abs(t[2]) < .9 else [0, 1, 0]))
        b = cross(t, n)
        r = radius[i] if isinstance(radius, list) else radius
        for j in range(sides):
            theta = 2*math.pi*j/sides
            vertices.append(add(point, add(mul(n, r*math.cos(theta)), mul(b, r*math.sin(theta)))))
    for i in range(count if closed else count-1):
        for j in range(sides):
            a, b = i*sides+j, i*sides+(j+1) % sides
            c, d = ((i+1) % count)*sides+(j+1) % sides, ((i+1) % count)*sides+j
            faces.extend([[a, b, c], [a, c, d]])
    if not closed:
        vertices.extend([points[0], points[-1]])
        for j in range(sides):
            faces.extend([[len(vertices)-2, (j+1) % sides, j],
                          [len(vertices)-1, (count-1)*sides+j, (count-1)*sides+(j+1) % sides]])
    return {"vertices": vertices, "faces": faces}


def ellipsoid(center, radii, rings=10, sides=16):
    vertices = [add(center, [0, 0, radii[2]])]
    for i in range(1, rings):
        phi = math.pi*i/rings
        for j in range(sides):
            t = 2*math.pi*j/sides
            vertices.append(add(center, [radii[0]*math.sin(phi)*math.cos(t), radii[1]*math.sin(phi)*math.sin(t), radii[2]*math.cos(phi)]))
    bottom = len(vertices)
    vertices.append(add(center, [0, 0, -radii[2]]))
    faces = []
    for j in range(sides):
        faces.append([0, 1+j, 1+(j+1) % sides])
        faces.append([bottom, 1+(rings-2)*sides+(j+1) % sides, 1+(rings-2)*sides+j])
    for i in range(rings-2):
        for j in range(sides):
            a, b = 1+i*sides+j, 1+i*sides+(j+1) % sides
            c, d = 1+(i+1)*sides+(j+1) % sides, 1+(i+1)*sides+j
            faces.extend([[a, d, c], [a, c, b]])
    return {"vertices": vertices, "faces": faces}


def ellipse(center, rx, ry, plane="xy", samples=40):
    points = []
    for i in range(samples):
        t = 2*math.pi*i/samples
        q = [rx*math.cos(t), ry*math.sin(t)]
        offset = [q[0], q[1], 0] if plane == "xy" else ([q[0], 0, q[1]] if plane == "xz" else [0, q[0], q[1]])
        points.append(add(center, offset))
    return points


def torso_mesh(stations, sides=36):
    vertices, faces = [], []
    for z, rx, ry, cy in stations:
        for j in range(sides):
            t = 2*math.pi*j/sides
            vertices.append([rx*math.cos(t), cy+ry*math.sin(t), z])
    for i in range(len(stations)-1):
        for j in range(sides):
            a, b = i*sides+j, i*sides+(j+1) % sides
            c, d = (i+1)*sides+(j+1) % sides, (i+1)*sides+j
            faces.extend([[a, b, c], [a, c, d]])
    for first in (True, False):
        i = 0 if first else len(stations)-1
        z, _, _, cy = stations[i]
        center = len(vertices); vertices.append([0, cy, z])
        for j in range(sides):
            a, b = i*sides+j, i*sides+(j+1) % sides
            faces.append([center, b, a] if first else [center, a, b])
    return {"vertices": vertices, "faces": faces}


def build_armature(height_mm=None, refinement=1.0, arm_drop_deg=75.0, span_equals_height=False):
    """Build a complete, bilateral mechanical form-study assembly.

    Span closure uses external segment ratios, not asserted skeletal lengths.
    This explicit station mapping is not counted as an aesthetic adjustment.
    """
    for label, value, low, high in (("refinement", refinement, 0, 1), ("arm drop", arm_drop_deg, 0, 80)):
        if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f"invalid {label}")
    ref = reference_statistics()
    m = {key: row["mean_mm"] for key, row in ref["statistics"].items()}
    height = m["stature"] if height_mm is None else height_mm
    if type(height) not in (int, float) or not math.isfinite(height) or not 1400 <= height <= 1900:
        raise ValueError("height must be 1400..1900 mm")
    profile = read(PROJECT / "profiles/f28.json")
    adjustments = profile["aesthetic_adjustments_percent"]
    if any(abs(v) > 3 for v in adjustments.values()):
        raise ValueError("refinement exceeds the authorized slight adjustment")
    scale = height/m["stature"]
    m = {k: v*scale for k, v in m.items()}
    shoulder = m["biacromialbreadth"]*(1+adjustments["shoulder"]*refinement/100)
    pelvis = m["bicristalbreadth"]*(1+adjustments["hip"]*refinement/100)
    waist = m["waistbreadth"]*(1+adjustments["waist"]*refinement/100)
    d = profile["design_choices_mm"]
    shoulder_x = shoulder/2 - d["shoulder_center_medial_offset"]*scale
    shoulder_z = m["acromialheight"]-14*scale
    hip_x = pelvis*d["hip_center_spacing_fraction_of_bicristal_breadth"]/2
    hip_z = m["trochanterionheight"]+d["hip_center_height_above_trochanterion"]*scale
    knee_z, ankle_z = m["lateralfemoralepicondyleheight"], d["ankle_center_height"]*scale
    target_span = height if span_equals_height else m["span"]
    proxies = [m["acromionradialelength"], m["radialestylionlength"], m["handlength"]]
    closure = (target_span/2-shoulder_x)/sum(proxies)
    upper, fore, hand = [v*closure for v in proxies]
    parts, landmarks, maquette_formers, maquette_members = [], {}, [], []

    def part(pid, name, region, material, mesh, source="proposed mechanical geometry", **details):
        vertices = [[round(v, 5) for v in p] for p in mesh["vertices"]]
        bounds = {"min": [min(v[i] for v in vertices) for i in range(3)],
                  "max": [max(v[i] for v in vertices) for i in range(3)]}
        parts.append({"id": pid, "name": name, "region": region, "material": material,
                      "authority": "project design / form study", "source": source,
                      "bounds_mm": bounds, "vertices": vertices, "faces": mesh["faces"], **details})

    def rod(pid, name, region, a, b, radius, material="redwood"):
        part(pid, name, region, material, tube([a, b], radius),
             endpoints_mm=[a, b], center_distance_mm=round(math.dist(a, b), 5))

    head_h = d["head_height"]*scale
    head_center = [0, 0, height-head_h/2]
    rx, ry, rz = m["headbreadth"]/2, m["headlength"]/2, head_h/2
    for i, plane in enumerate(("xz", "yz")):
        part(f"head_arch_{i}", "Cranial arch " + plane.upper(), "head", "brass",
             tube(ellipse(head_center, (rx if plane == "xz" else ry)-3*scale, rz-3*scale, plane), 3*scale, True))
    for i, f in enumerate((-.4, 0, .45)):
        part(f"head_band_{i}", f"Cranial former {i+1}", "head", "redwood",
             tube(ellipse(add(head_center, [0, 0, rz*f]), rx*math.sqrt(1-f*f), ry*math.sqrt(1-f*f)), 3*scale, True))
    part("head_envelope", "Head envelope", "envelope", "ivory", ellipsoid(head_center, [rx, ry, rz]),
         source="ANSUR head breadth/depth; proposed 220 mm chin–crown height")

    # Preserve the separate earlier plywood form-study schedule, not skeletal ribs.
    rib_low, rib_high = m["tenthribheight"]-20*scale, shoulder_z-70*scale
    for i in range(7):
        t = i/6
        z = rib_low+(rib_high-rib_low)*t
        breadth = m["chestbreadth"]*(.72+.28*math.sin(math.pi*(.12+.78*t)))
        depth = m["chestdepth"]*(.62+.12*math.sin(math.pi*t))
        maquette_formers.append({"id": f"rib_{i+1}", "former_mm": [z, breadth, depth]})
    rod("shoulder_bridge", "Thoracic shoulder bridge", "shoulders", [-shoulder_x, -20*scale, shoulder_z], [shoulder_x, -20*scale, shoulder_z], 9*scale)
    for side, s in (("R", 1), ("L", -1)):
        a, b = [s*34*scale, 32*scale, shoulder_z+20*scale], [s*shoulder_x, 0, shoulder_z]
        part(f"clavicle_{side}", f"{side} floating clavicular link", "shoulders", "brass", tube([a, [s*shoulder_x*.65, 24*scale, shoulder_z+14*scale], b], 6*scale))
    for side, s in (("R", 1), ("L", -1)):
        outer_hip = m["hipbreadth"]*(1+.02*refinement)/2
        form_curve = [[s*hip_x, 0, hip_z-35*scale],
                      [s*outer_hip*.92, 0, hip_z+10*scale],
                      [s*outer_hip, -10*scale, hip_z+55*scale],
                      [s*pelvis*.48, -22*scale, m["iliocristaleheight"]]]
        part(f"hip_form_{side}", f"{side} outer hip form rail", "pelvis", "copper", tube(form_curve, 4*scale),
             source="measured external hip breadth; proposed form-support path")
        part(f"chest_form_{side}", f"{side} anterior chest form arch", "thorax", "copper",
             tube(ellipse([s*m["chestbreadth"]*.25, m["chestdepth"]*.39, m["chestheight"]],
                          m["chestdepth"]*.16, 52*scale, "yz"), 3*scale, True),
             source="external chest depth/girth guide; proposed anterior form arch, not anatomy or measured bone")

    for side, s in (("R", 1), ("L", -1)):
        angle = math.radians(arm_drop_deg)
        direction = [s*math.cos(angle), 0, -math.sin(angle)]
        shoulder_p = [s*shoulder_x, 0, shoulder_z]
        elbow = add(shoulder_p, mul(direction, upper))
        wrist = add(elbow, mul(direction, fore))
        finger_tip = add(wrist, mul(direction, hand))
        hip, knee, ankle = [s*hip_x, 0, hip_z], [s*60*scale, 8*scale, knee_z], [s*62*scale, 0, ankle_z]
        for joint_name, point, radius in (("shoulder", shoulder_p, 19), ("elbow", elbow, 14), ("wrist", wrist, 11), ("hip", hip, 20), ("knee", knee, 17), ("ankle", ankle, 13)):
            landmarks[f"{joint_name}_{side}"] = point
            part(f"{joint_name}_joint_{side}", f"{side} {joint_name} coupling envelope", "shoulders" if joint_name == "shoulder" else ("arms" if joint_name in ("elbow", "wrist") else "legs"), "brass", ellipsoid(point, [radius*scale]*3), source="proposed center; coupling envelope only, no internal mechanism")
        rod(f"upper_arm_{side}", f"{side} upper-arm member", "arms", shoulder_p, elbow, [13*scale, 10*scale])
        rod(f"femur_{side}", f"{side} thigh member", "legs", hip, knee, [17*scale, 13*scale])
        rod(f"tibia_{side}", f"{side} shank member", "legs", knee, ankle, [12*scale, 9*scale])
        rod(f"fibular_{side}", f"{side} auxiliary shank rail", "legs", add(knee, [s*22*scale, -8*scale, -16*scale]), add(ankle, [s*14*scale, -8*scale, 14*scale]), 4*scale, "copper")
        maquette_members.extend({key: p[key] for key in ("id", "region", "endpoints_mm")}
                                for p in parts if p["id"] in
                                (f"upper_arm_{side}", f"femur_{side}", f"tibia_{side}", f"fibular_{side}"))
        # Keep the earlier stencil's order and single forearm carrier exactly.
        maquette_members.insert(len(maquette_members)-3,
                                {"id": f"forearm_{side}", "region": "arms", "endpoints_mm": [elbow, wrist]})
        palm_end = add(wrist, mul(direction, hand*.56))
        rod(f"palm_{side}", f"{side} palm carrier", "hands", wrist, palm_end, [14*scale, 18*scale])
        lateral = [s*math.sin(angle), 0, math.cos(angle)]
        for i, length_factor in enumerate((.76, .94, 1.0, .94, .78)):
            spread = (i-2)*m["handbreadth"]*.16
            start = add(palm_end, mul(lateral, spread))
            tip = add(add(wrist, mul(direction, hand*(.56+.44*length_factor))), mul(lateral, spread))
            mid = add(start, mul(sub(tip, start), .55))
            rod(f"finger_{side}_{i}", f"{side} digit {i+1}", "hands", start, mid, 3.5*scale)
            rod(f"finger_tip_{side}_{i}", f"{side} digit {i+1} distal", "hands", mid, tip, 2.8*scale, "brass")
        foot_center = [ankle[0], m["footlength"]*.28, 34*scale]
        part(f"foot_{side}", f"{side} foot form block", "feet", "redwood", ellipsoid(foot_center, [m["footbreadthhorizontal"]/2, m["footlength"]/2, 34*scale]), source="measured heel–toe length and foot breadth; proposed ellipsoidal form")
        landmarks[f"fingertip_{side}"] = finger_tip

    # Smooth soft-form guide, explicitly an interpolated envelope, not scan data.
    def girth_axes(girth, breadth, depth):
        a, b = breadth/2, depth/2
        circumference = math.pi*(3*(a+b)-math.sqrt((3*a+b)*(a+3*b)))
        return a*girth/circumference, b*girth/circumference
    chest_axes = girth_axes(m["chestcircumference"], m["chestbreadth"], m["chestdepth"])
    waist_axes = girth_axes(m["waistcircumference"]*(1-.03*refinement), waist, m["waistdepth"]*(1-.03*refinement))
    hip_axes = girth_axes(m["buttockcircumference"]*(1+.02*refinement), m["hipbreadth"]*(1+.02*refinement), m["hipbreadth"]*.72)
    stations = [[hip_z-90*scale, hip_axes[0]*.72, hip_axes[1]*.72, 0],
                [hip_z-25*scale, hip_axes[0], hip_axes[1], -12*scale],
                [hip_z+55*scale, hip_axes[0]*.94, hip_axes[1]*.95, -8*scale],
                [m["waistheightomphalion"]+35*scale, waist_axes[0], waist_axes[1], 0],
                [m["tenthribheight"]+60*scale, chest_axes[0]*.88, chest_axes[1]*.86, 7*scale],
                [m["chestheight"], chest_axes[0], chest_axes[1], 15*scale],
                [shoulder_z-85*scale, chest_axes[0]*.85, chest_axes[1]*.82, 0],
                [shoulder_z-5*scale, shoulder_x*.9, 62*scale, 0]]
    stations.sort(key=lambda row: row[0])
    part("torso_envelope", "Interpolated female form envelope", "envelope", "ivory", torso_mesh(stations), source="ANSUR marginal girths; ellipse fit and inter-level interpolation are proposed form design")
    landmarks["vertex"] = [0, 0, height]
    landmarks["floor"] = [0, 0, 0]
    bone_report, bone_anchors = build_core(parts, part,
        (scale, height, m, shoulder_x, shoulder_z, hip_x, hip_z, pelvis), profile["bone_proxy_layout"])
    return {"id": "PHY_F28_R1", "name": "Female 28 / reference armature", "units": "mm",
            "frame": "+x subject-right, +y anterior, +z superior", "age_years": 28,
            "height_mm": height, "t_pose_span_mm": target_span, "refinement": refinement,
            "arm_drop_deg": arm_drop_deg, "scale_from_mean": scale,
            "canon_effect": "none", "canon_identity_comparison_only": span_equals_height,
            "fabrication_released": False, "physical_evidence_complete": False,
            "reference": ref, "parts": parts, "landmarks": landmarks,
            "bone_equivalence": bone_report, "bone_anchors_mm": bone_anchors,
            "bone_proxy_layout": profile["bone_proxy_layout"],
            "bone_proxy_layout_sha256": digest(profile["bone_proxy_layout"]),
            "maquette_formers": maquette_formers, "maquette_members": maquette_members,
            "design_datums": {"hip_center_spacing_mm": 2*hip_x, "shoulder_center_spacing_mm": 2*shoulder_x,
                              "arm_chain_closure_scale": closure, "upper_arm_station_mm": upper,
                              "forearm_station_mm": fore, "hand_station_mm": hand,
                              "shoulder_z_mm": shoulder_z, "hip_z_mm": hip_z,
                              "knee_z_mm": knee_z, "ankle_z_mm": ankle_z},
            "aesthetic_adjustments_percent": {k: v*refinement for k, v in adjustments.items()},
            "limitations": ["External surface measurements do not specify bone lengths or 3D joint centers.",
                            "The arm station mapping preserves measured span using segment ratios; it is not anatomical endpoint adoption.",
                            "71 core bones have individual provisional proxies; 128 bones remain grouped forms and seven are unrepresented.",
                            "Count/adjacency is reference-grounded; bone dimensions, morphology and joints are unverified design proposals.",
                            "Hardware, support rails and interpolated envelopes do not count as bones. Thirteen facial source records remain missing.",
                            "The A0 shoulder article is a separate, unscaled bench assembly; no automatic whole-body integration.",
                            "The quarter-scale passive maquette has separate assembly and material instructions.",
                            "Full-scale load paths, interfaces, actuation, balance and physical qualification remain open."]}


def write_stl(parts, path, scale=1):
    """ASCII STL in mm; selected armature meshes, not qualified machining geometry."""
    with Path(path).open("w", encoding="ascii") as handle:
        handle.write("solid PHY_REFERENCE_NOT_STRUCTURAL_RELEASE\n")
        for part in parts:
            for face in part["faces"]:
                a, b, c = [mul(part["vertices"][j], scale) for j in face]
                normal = cross(sub(b, a), sub(c, a))
                try:
                    normal = unit(normal)
                except ValueError:
                    raise ValueError("degenerate export face in " + part["id"])
                handle.write(" facet normal " + " ".join(f"{v:.7g}" for v in normal) + "\n  outer loop\n")
                for vertex in (a, b, c):
                    handle.write("   vertex " + " ".join(f"{v:.7g}" for v in vertex) + "\n")
                handle.write("  endloop\n endfacet\n")
        handle.write("endsolid PHY_REFERENCE_NOT_STRUCTURAL_RELEASE\n")
