#!/usr/bin/env python3
"""Build an offline PHY Studio, meshes, reference report and passive maquette.

Viewer rebuild: npm ci --prefix studio && npm run build --prefix studio
Mesh rebuild: python bin/export_phy_studio.py [--output DIR] [--without-a0]
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import shutil
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from skeleton.visualization.studio_model import build_armature, reference_statistics, write_stl
from skeleton.visualization.bone_equivalence import report_markdown


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


def a0_model():
    """Tessellate the current located A0 B-rep, with no geometric reinterpretation."""
    exporter = load_module("phy_a0_export", ROOT / "bin/export_t56_a0_article.py")
    design = exporter.load_design()
    instances = exporter.located_instances(exporter.build_parts(design), design)
    parts = []
    for iid, pid, workplane in instances:
        shape = workplane.val()
        if not shape.isValid() or shape.Volume() <= 0:
            raise ValueError("invalid A0 B-rep " + iid)
        box = shape.BoundingBox()
        vertices, faces = shape.tessellate(.6, .18)
        material = "steel" if any(k in pid for k in ("shaft", "key", "bolt", "retainer", "bearing_envelope", "tether")) else ("acetal" if "pad" in pid else "aluminum")
        parts.append({"id": iid, "name": iid.replace("_", " ").title(), "part_id": pid,
                      "region": "fixture" if "fixture" in iid or "standoff" in iid else "mechanism",
                      "material": material, "authority": "A0-R1 located shop-review B-rep",
                      "source": "bin/export_t56_a0_article.py → located_instances",
                      "bounds_mm": {"min": [box.xmin, box.ymin, box.zmin], "max": [box.xmax, box.ymax, box.zmax]},
                      "volume_mm3": round(shape.Volume(), 6),
                      "vertices": [[round(v.x, 5), round(v.y, 5), round(v.z, 5)] for v in vertices],
                      "faces": [list(f) for f in faces]})
    packet = ROOT / "PROJECTS/T56_CARBON/fabrication/a0_packet"
    return {"id": "A0_R1", "name": "A0-R1 / shoulder bench article", "units": "mm",
            "frame": design["frame_id"], "parts": parts, "landmarks": {}, "canon_effect": "none",
            "fabrication_released": False, "physical_evidence_complete": False,
            "motion": exporter.motion_report(design),
            "bom": [dict(zip(["item_id", "qty", "description", "material", "note"], row)) for row in exporter.bom_rows(design)],
            "convergence": json.loads((packet / "analysis/A0_CONVERGENCE.json").read_text()),
            "limitations": ["Exact current 35-instance neutral B-rep, tessellated for viewing.",
                            "19 distinct CAD parts; 24 BOM lines. It is a separate, unscaled bench article.",
                            "3.25 kN screened bearing reaction exceeds the provisional 3.0 kN threshold.",
                            "Supplier interfaces, stop-pad response, continuous clearance, independent review and physical bench records remain open."]}


def source_kit():
    exporter = load_module("phy_component_export", ROOT / "bin/export_component_3d.py")
    specs, omitted = exporter.component_specs()
    parts = []
    for index, spec in enumerate(specs):
        shape = exporter.build_shape(spec)
        vertices, faces = shape.tessellate(.8, .2)
        # Gallery transforms are display layout only. Original local dimensions remain unchanged.
        x, z = (index % 6)*140-350, (index // 6)*480
        vs = [[round(v.x+x, 5), round(v.y, 5), round(v.z+z, 5)] for v in vertices]
        parts.append({"id": spec["id"], "name": spec["name"], "region": "reference",
                      "material": "brass" if spec["kind"] == "bushing" else "redwood",
                      "source": spec["source"], "authority": spec["authority"],
                      "display_transform_only": [x, 0, z], "dimensions_mm": spec["xyz_mm"],
                      "bounds_mm": {"min": [min(v[i] for v in vs) for i in range(3)], "max": [max(v[i] for v in vs) for i in range(3)]},
                      "vertices": vs, "faces": [list(f) for f in faces]})
    return {"id": "REFERENCE_KIT", "name": "Legacy / source component library", "parts": parts,
            "units": "mm", "landmarks": {}, "fabrication_released": False,
            "limitations": ["18 existing source-based component envelopes, displayed on a gallery grid.",
                            "Gallery positions are not anatomical placement or an assembled armature."], "omitted": omitted}


def maquette(model, output):
    """Quarter-scale, supported plywood form study. Exact cut paths and hole datums.

    This is a separate embodiment: planar pinned limbs, solid torso formers,
    a rear dowel and a base. It does not miniaturize the A0 joint mechanism.
    """
    output.mkdir(parents=True, exist_ok=True)
    scale, thickness, pin_hole, post_hole = .25, 3, 1.6, 4.2
    d = model["design_datums"]
    rows, shapes = [], []

    def add(pid, name, w, h, svg, qty=1, note=""):
        for i in range(qty):
            shapes.append((f"{pid}-{i+1}", name, w, h, svg))
        rows.append({"id": pid, "qty": qty, "description": name, "width_mm": round(w, 4), "length_mm": round(h, 4), "stock": "3 mm birch plywood", "note": note})

    def link(pid, name, center, qty=2):
        length, w = center*scale, 12
        svg = f'<rect x="0" y="0" width="{w}" height="{length+12}" rx="5"/>'
        svg += ''.join(f'<circle cx="6" cy="{y}" r="{pin_hole/2}"/>' for y in (6, length+6))
        add(pid, name, w, length+12, svg, qty, f"Hole centers {length:.3f} mm; Ø1.6 mm for 1.5 mm wire pin")

    link("M-UPPER", "upper-arm link", d["upper_arm_station_mm"])
    link("M-FORE", "forearm link", d["forearm_station_mm"])
    link("M-HAND", "hand form link", d["hand_station_mm"])
    link("M-THIGH", "thigh link", math.dist(model["landmarks"]["hip_R"], model["landmarks"]["knee_R"]))
    link("M-SHANK", "shank link", math.dist(model["landmarks"]["knee_R"], model["landmarks"]["ankle_R"]))
    former_rows = []
    for part in model["maquette_formers"]:
        if "former_mm" in part:
            z, w, h = part["former_mm"]
            w, h = w*scale, h*scale
            svg = f'<ellipse cx="{w/2}" cy="{h/2}" rx="{w/2}" ry="{h/2}"/><circle cx="{w/2}" cy="{h/2-8}" r="{post_hole/2}"/>'
            add("M-"+part["id"].upper(), "thoracic former "+part["id"].split("_")[1], w, h, svg,
                note=f"Lower face Z={z*scale:.3f} mm above base top; post axis y=-8 mm")
            former_rows.append({"part": "M-"+part["id"].upper(), "lower_face_z_mm": round(z*scale, 4)})
    pelvis_w = model["reference"]["statistics"]["bicristalbreadth"]["mean_mm"]*model["scale_from_mean"]*1.02*scale
    for i, z in enumerate((d["hip_z_mm"]-5, d["hip_z_mm"]+60)):
        h = 36
        svg = f'<ellipse cx="{pelvis_w/2}" cy="18" rx="{pelvis_w/2}" ry="18"/><circle cx="{pelvis_w/2}" cy="10" r="2.1"/>'
        add(f"M-PELVIS-{i+1}", "pelvic former", pelvis_w, h, svg, note=f"Lower face Z={z*scale:.3f} mm")
        former_rows.append({"part": f"M-PELVIS-{i+1}", "lower_face_z_mm": round(z*scale, 4)})
    shoulder_w = d["shoulder_center_spacing_mm"]*scale+12
    svg = f'<rect width="{shoulder_w}" height="16" rx="4"/>' + ''.join(f'<circle cx="{x}" cy="8" r=".8"/>' for x in (6, shoulder_w-6))
    add("M-SHOULDER", "shoulder bridge", shoulder_w, 16, svg, note=f"Horizontal shoulder pin axis Z={d['shoulder_z_mm']*scale:.3f} mm; wire-wrap to rear post")
    hip_w = d["hip_center_spacing_mm"]*scale+12
    add("M-HIP", "hip bridge", hip_w, 16, f'<rect width="{hip_w}" height="16" rx="4"/><circle cx="6" cy="8" r=".8"/><circle cx="{hip_w-6}" cy="8" r=".8"/>', note=f"Hip pin axis Z={d['hip_z_mm']*scale:.3f} mm; wire-wrap to rear post")
    add("M-BASE", "support base", 100, 80, '<rect width="100" height="80" rx="6"/><circle cx="50" cy="32" r="2.1"/>', qty=2, note="Laminate two layers. Post passes through both: engagement 6 mm. Add scrap reinforcement if needed.")
    ref = model["reference"]["statistics"]
    for side in ("R", "L"):
        w, h = ref["footbreadthhorizontal"]["mean_mm"]*scale, ref["footlength"]["mean_mm"]*scale
        add("M-FOOT-"+side, "foot silhouette", w, h, f'<ellipse cx="{w/2}" cy="{h/2}" rx="{w/2}" ry="{h/2}"/>', note="Glue ankle link to footprint; intended fixed standing maquette")
    w, h = ref["headbreadth"]["mean_mm"]*scale, 220*model["scale_from_mean"]*scale
    add("M-HEAD", "frontal head silhouette", w, h, f'<ellipse cx="{w/2}" cy="{h/2}" rx="{w/2}" ry="{h/2}"/>', note="Glue to upper post. Proposed head height, not measured chin–crown.")

    # Shelf pack with exact physical dimensions; labels are a separate non-cut layer.
    pages, items, x, y, row_h = [], [], 10., 22., 0.
    def finish():
        if items:
            pages.append(items.copy()); items.clear()
    for pid, name, w, h, shape in shapes:
        if x+w > 410:
            x, y, row_h = 10., y+row_h+9, 0.
        if y+h > 277:
            finish(); x, y, row_h = 10., 22., 0.
        items.append((pid, name, x, y, w, h, shape)); x += w+8; row_h = max(row_h, h)
    finish()
    for i, items in enumerate(pages):
        cuts = ''.join(f'<g transform="translate({x},{y})">{s}</g>' for _, _, x, y, _, _, s in items)
        labels = ''.join(f'<text x="{x}" y="{y+h+3}" font-size="2.2">{pid}</text>' for pid, _, x, y, _, h, _ in items)
        svg = f'<svg xmlns="http://www.w3.org/2000/svg" width="420mm" height="297mm" viewBox="0 0 420 297"><title>PHY F28 quarter-scale maquette sheet {i+1}</title><g id="CUT" fill="none" stroke="#e00000" stroke-width="0.08">{cuts}</g><g id="ENGRAVE-OR-IGNORE" fill="#333" font-family="monospace"><text x="10" y="9" font-size="4">PHY F28 • 1:4 PASSIVE MAQUETTE • 3 mm plywood • mm • sheet {i+1}</text><text x="10" y="15" font-size="2.8">CUT layer only. No kerf compensation. Verify the 50 mm calibration line.</text>{labels}<path d="M10 286 H60" fill="none" stroke="#333" stroke-width=".2"/><text x="10" y="291" font-size="2.5">50 mm calibration • do not scale to page</text></g></svg>\n'
        (output / f"cut-sheet-{i+1}.svg").write_text(svg, encoding="utf-8")
    rows.extend([{"id": "M-POST", "qty": 1, "description": "rear support post", "width_mm": 4, "length_mm": round((model["height_mm"]-90)*scale+6, 4), "stock": "4 mm hardwood dowel", "note": "Includes 6 mm base engagement. Sand/test Ø4.2 coupon before assembly."},
                 {"id": "M-PINS", "qty": 10, "description": "limb pins / bridge lashing", "width_mm": 1.5, "length_mm": 12, "stock": "1.5 mm soft copper wire", "note": "Bend/capture both ends. No load rating. Adjust wire length to actual stack."}])
    with (output / "BOM.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]), lineterminator="\n"); writer.writeheader(); writer.writerows(rows)
    manifest = {"scale": scale, "height_mm": model["height_mm"]*scale, "status": "passive_supported_form_study",
                "stock_thickness_mm": thickness, "cut_sheets": len(pages), "cut_parts": len(shapes),
                "kerf_compensation_mm": 0, "pin_hole_mm": pin_hole, "post_hole_mm": post_hole,
                "material_qualification": "none; not a structural or actuated armature",
                "former_positions": former_rows, "bill_of_materials": rows}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
    # Front assembly stencil uses datum points; y depths are shown by former schedule.
    lines = []
    for part in model["maquette_members"]:
        if "endpoints_mm" in part and part["region"] in ("arms", "legs"):
            a, b = part["endpoints_mm"]
            lines.append(f'<line x1="{125+a[0]*scale}" y1="{440-a[2]*scale}" x2="{125+b[0]*scale}" y2="{440-b[2]*scale}"/>')
    labels = ''.join(f'<text x="175" y="{440-d[key]*scale}" font-size="3.5">{name}: {d[key]*scale:.2f} mm</text>' for key, name in (("shoulder_z_mm", "Shoulder"), ("hip_z_mm", "Hip"), ("knee_z_mm", "Knee"), ("ankle_z_mm", "Ankle")))
    forms = []
    for p in model["maquette_formers"]:
        if "former_mm" in p:
            z, w, _ = p["former_mm"]
            forms.append(f'<rect x="{125-w*scale/2}" y="{440-z*scale-3}" width="{w*scale}" height="3"/>')
    for z in (d["hip_z_mm"]-5, d["hip_z_mm"]+60):
        forms.append(f'<rect x="{125-pelvis_w/2}" y="{440-z*scale-3}" width="{pelvis_w}" height="3"/>')
    forms.append(f'<ellipse cx="125" cy="{440-(model["height_mm"]-110*model["scale_from_mean"])*scale}" rx="{ref["headbreadth"]["mean_mm"]*scale/2}" ry="{110*model["scale_from_mean"]*scale}"/>')
    pins = ''.join(f'<circle cx="{125+p[0]*scale}" cy="{440-p[2]*scale}" r="1.5"/>' for k, p in model["landmarks"].items() if k.split('_')[0] in ("shoulder", "elbow", "wrist", "hip", "knee", "ankle"))
    svg = '<svg xmlns="http://www.w3.org/2000/svg" width="260mm" height="470mm" viewBox="0 0 260 470"><rect width="260" height="470" fill="white"/><g font-family="monospace" fill="#263832"><text x="10" y="12" font-size="5">PHY F28 • 1:4 assembly stencil</text><text x="10" y="21" font-size="3">Passive display only • all dimensions mm • print actual size</text>'+labels+'</g><g fill="#ece5d7" stroke="#947453" stroke-width=".4">'+''.join(forms)+'</g><g fill="none" stroke="#947453" stroke-width="3">'+''.join(lines)+'</g><g fill="#b88b50">'+pins+'</g><line x1="10" y1="440" x2="240" y2="440" stroke="#444"/><line x1="125" y1="48" x2="125" y2="440" stroke="#888" stroke-dasharray="3,3"/><text x="10" y="458" font-family="monospace" font-size="3">Rear post at Y=-8 mm. Former depth schedule: manifest.json.</text></svg>\n'
    (output / "assembly-stencil.svg").write_text(svg)
    (output / "BUILD.md").write_text(f"""# PHY F28 quarter-scale passive maquette

This is a {manifest['height_mm']:.2f} mm tall, externally supported **form study**.
It is the first inexpensive physical demonstration. It is not a load-bearing,
motorized or full-scale armature, and does not use the A0 shoulder mechanism.

## Material and cuts

- {len(pages)} A3 cut sheets, actual size, 3 mm birch plywood; CUT paths only.
- 4 mm hardwood dowel: cut {rows[-2]['length_mm']:.2f} mm, including 6 mm base engagement.
- Soft 1.5 mm copper wire; nominal ten 12 mm pins plus bridge lashing.
- Wood glue for fixed connections. Tape/spacer stock for locating the formers.

The two base plates laminate into a 6 mm base. The support post is at
(x=0, y=-8) mm relative to the body center; all former holes use that datum.
The SVG holes are nominal Ø4.2 and Ø1.6 mm. No kerf correction is applied.
First cut a scrap coupon with those holes, measure stock and dowel, and adjust
the cutter offset before cutting the sheets. Confirm the 50 mm calibration line.

## Assembly

1. Laminate the two base plates, align the holes and glue the post through both.
2. Mark the post heights in manifest.json (former lower faces above base top).
3. Slide solid transverse pelvis and thorax formers onto the post and fix at
   those marks with glue or taped collars. They are solid silhouette panels,
   not scaled replicas of the curved full-body reference ribs.
4. Lash/glue the hip and shoulder bridges to the post at the pin-axis heights
   in BOM.csv. The broad face of each bridge lies in the frontal XZ plane.
5. Connect the two sets of thigh/shank and upper-arm/forearm/hand links with
   copper pins; capture both ends. These are display hinges, not bearings.
   Set the limb axes against assembly-stencil.svg at 100% print scale.
6. Glue ankle links to the foot silhouettes and the head silhouette to the
   upper post. Use spacers/scrap tabs at the bridges as needed for this glued
   model; these attachments carry no functional load.
7. Check height, shoulder/hip spacing and bilateral alignment with a ruler.
   Record observations; no physical build or dimensional inspection is claimed.

Use the rear post to support the display. No free-standing balance, joint torque,
strength, actuator fit or full-scale manufacturability follows from this model.
The maquette uses the refined reference height and A-pose. Other Studio states
do not alter these cut files; regenerate an explicit profile before new cuts.
""", encoding="utf-8")
    return manifest


def export(output, include_a0=True):
    output.mkdir(parents=True, exist_ok=True)
    reference = reference_statistics()
    mean = build_armature(refinement=0)
    refined = build_armature()
    sophy = build_armature(height_mm=1676.4, span_equals_height=True)
    audit = refined["bone_equivalence"]
    (output / "BONE_EQUIVALENCE.json").write_text(json.dumps(audit, indent=2)+"\n")
    (output / "BONE_EQUIVALENCE.md").write_text(report_markdown(audit), encoding="utf-8")
    layout_report = ["# Core bone proxy layout / CORE_V1", "",
                     "Project-local provisional placement, not osteometry, joint anatomy or canon adoption.",
                     "Values are mm at the cohort mean stature unless the key states fraction, degrees or samples.",
                     "Mean/refined/SOPHY overlays scale these choices uniformly; all dimensional fidelity is unverified.",
                     "Source: `PROJECTS/PHY_F28/profiles/f28.json#/bone_proxy_layout`", "",
                     "Layout SHA-256: `"+refined["bone_proxy_layout_sha256"]+"`", "",
                     "| Input | Governing value |", "| --- | --- |"]
    layout_report += ["| "+key+" | `"+json.dumps(value, sort_keys=True)+"` |"
                      for key, value in refined["bone_proxy_layout"].items()]
    layout_report += ["", "External station inputs remain in the hash-pinned reference projection and `design_choices_mm`.",
                      "The retained humerus/femur/tibia/fibula/clavicle proxies are station-based forms, not measured bones.",
                      "No clearances, structural sections, joint interfaces or manufacturing release are asserted."]
    (output / "CORE_PROXY_LAYOUT.md").write_text("\n".join(layout_report)+"\n", encoding="utf-8")
    hand_report = ["# Individual hand proxy layout / HAND_V1", "",
                   "27 identities per hand: eight carpals, five metacarpals and 14 phalanges.",
                   "Static radial/palmar thumb review pose; working opposition and all joint interfaces remain unresolved.",
                   "Numeric values are project-local provisional display choices, not measured bone lengths or canon.",
                   "Source: `PROJECTS/PHY_F28/profiles/f28.json#/hand_proxy_layout`", "",
                   "Layout SHA-256: `"+refined["hand_proxy_layout_sha256"]+"`", "",
                   "The wrist-relative frame follows the distal arm, radial radius side and anterior palmar direction.",
                   "Left hand geometry, winding, anchors and topology IDs reflect the authored right side.", "",
                   "| Input | Governing value |", "| --- | --- |"]
    hand_report += ["| "+key+" | `"+json.dumps(value, sort_keys=True)+"` |"
                    for key, value in refined["hand_proxy_layout"].items()]
    hand_report += ["", "External hand breadth and span-derived hand station size this display layout only.",
                    "Intentional gaps separate display segments; they are not qualified joint fits or clearances.",
                    "Carpal row order and selected chain parents describe partial topology, not a complete articulation graph.",
                    "Dimensional fidelity unverified; physical evidence unmeasured; fabrication release false."]
    (output / "HAND_PROXY_LAYOUT.md").write_text("\n".join(hand_report)+"\n", encoding="utf-8")
    foot_report = ["# Individual foot proxy layout / FOOT_V1", "",
                   "26 identities per foot: seven tarsals, five metatarsals and 14 toe phalanges.",
                   "Static grounded display; measured surfaces, arches, ankle/toe mechanics and load paths remain unresolved.",
                   "Numeric values are project-local provisional display choices, not osteometry or canon.",
                   "Source: `PROJECTS/PHY_F28/profiles/f28.json#/foot_proxy_layout`", "",
                   "Layout SHA-256: `"+refined["foot_proxy_layout_sha256"]+"`", "",
                   "Right frame: +x lateral, +y anterior, +z above the floor, heel located behind the ankle station.",
                   "Hallux is medial. Left geometry, winding, anchors and topology IDs reflect authored right geometry.", "",
                   "| Input | Governing value |", "| --- | --- |"]
    foot_report += ["| "+key+" | `"+json.dumps(value, sort_keys=True)+"` |"
                    for key, value in refined["foot_proxy_layout"].items()]
    foot_report += ["", "External heel-to-toe length and breadth size a display envelope, not individual bone measurements.",
                    "Sections/elevations/gaps scale uniformly; tarsal horizontal sections use the external breadth/length.",
                    "Heel and nominal second-toe terminal station span the input length; tube ends are trimmed for display gaps.",
                    "This nominal station closure is not exact mesh-envelope conformity or anatomical endpoint adoption.",
                    "Selected chain parents describe partial topology, not a complete joint/contact graph.",
                    "Gaps are not qualified fits; no continuous clearance, gait, arch function or structural proof is asserted.",
                    "Dimensional fidelity unverified; physical evidence unmeasured; fabrication release false."]
    (output / "FOOT_PROXY_LAYOUT.md").write_text("\n".join(foot_report)+"\n", encoding="utf-8")
    skull_report = ["# Individual skull proxy layout / SKULL_V1", "",
                    "22 identities: eight cranial and 14 facial bones, including one adult mandible.",
                    "Closed shell patches, ellipsoids and open-centerline sweeps are provisional display geometry.",
                    "The shell patches have capped edges. No sutures, foramina, sinuses, teeth or articular surfaces are reproduced.",
                    "Source: `PROJECTS/PHY_F28/profiles/f28.json#/skull_proxy_layout`", "",
                    "Layout SHA-256: `"+refined["skull_proxy_layout_sha256"]+"`", "",
                    "Fractions use external half head breadth/depth and half of the proposed 220 mm head height.",
                    "ANSUR head length is anterior/posterior depth, not crown-to-chin height or a bone dimension.",
                    "Six midline and eight right-side proxies are authored; eight left counterparts reflect vertices, winding and selected adjacency IDs.",
                    "The separate mandible has bilateral temporal placement stations; no joint axis, fit or motion is implemented.", "",
                    "| Input | Governing value |", "| --- | --- |"]
    skull_report += ["| "+key+" | `"+json.dumps(value, sort_keys=True)+"` |"
                     for key, value in refined["skull_proxy_layout"].items()]
    skull_report += ["", "Selected adjacency and identity reference (not a measured morphology source):",
                     "https://openstax.org/books/anatomy-and-physiology-2e/pages/7-2-the-skull", "",
                     "Selected adjacency is not a complete suture/articulation graph or a mesh contact/clearance proof.",
                     "Nine skull identities have legacy source records; 13 facial records remain explicitly missing.",
                     "All 193 canonical source records are unchanged. Their existing dimensions do not qualify these proxies.",
                     "The five retained head arches/bands are form-study meshes with no bone correspondence.",
                     "Dimensional fidelity unverified; physical evidence unmeasured; fabrication release false."]
    (output / "SKULL_PROXY_LAYOUT.md").write_text("\n".join(skull_report)+"\n", encoding="utf-8")
    ear_hyoid_report = ["# Individual ear and hyoid proxy layout / EAR_HYOID_V1", "",
                        "Seven identities: malleus, incus and stapes on each side, plus one midline hyoid.",
                        "Together with the preserved 199 proxies, all 206 inventory identities are represented individually.",
                        "Count/distribution coverage does not establish measured morphology, working mechanics or fabrication readiness.",
                        "Source: `PROJECTS/PHY_F28/profiles/f28.json#/ear_hyoid_proxy_layout`", "",
                        "Layout SHA-256: `"+refined["ear_hyoid_proxy_layout_sha256"]+"`", "",
                        "Origins use the external half head breadth/depth and half the proposed 220 mm head height.",
                        "Local centerlines, offsets and sections are provisional millimeter display choices, scaled from the F28 mean height.",
                        "Three authored right ossicles reflect vertices, triangle winding, anchors and same-side chain IDs to the left.",
                        "Malleus and incus use tapered sweeps; stapes uses a closed stirrup loop with an open center.",
                        "The malleus–incus–stapes chain runs lateral to medial within the head display frame.",
                        "Temporal housing is a placement label, not an articulation, cavity, fit or clearance proof.",
                        "The tympanic membrane and oval window are endpoint labels; their geometry and mechanisms are unresolved.",
                        "The hyoid is one open U below the jaw, with posterior ends and no direct bone articulation.",
                        "Its muscle/ligament suspension, soft-tissue interfaces, hearing and swallowing mechanics remain unmodeled.", "",
                        "| Input | Governing value |", "| --- | --- |"]
    ear_hyoid_report += ["| "+key+" | `"+json.dumps(value, sort_keys=True)+"` |"
                         for key, value in refined["ear_hyoid_proxy_layout"].items()]
    ear_hyoid_report += ["", "Identity and selected topology references (not numerical geometry sources):",
                         "https://openstax.org/books/anatomy-and-physiology-2e/pages/14-1-sensory-perception",
                         "https://openstax.org/books/anatomy-and-physiology-2e/pages/7-2-the-skull", "",
                         "All seven identities have existing canonical source modules; their legacy dimension dictionaries remain unchanged.",
                         "Those records do not qualify these display meshes. All 193 source records are preserved; 13 facial records remain missing.",
                         "CORE_V1, HAND_V1, FOOT_V1 and SKULL_V1 geometry and inputs are preserved.",
                         "Dimensional fidelity unverified; physical evidence unmeasured; fabrication release false."]
    (output / "EAR_HYOID_PROXY_LAYOUT.md").write_text("\n".join(ear_hyoid_report)+"\n", encoding="utf-8")
    models = [dict(mean, id="F28_MEAN", name="F28 / arithmetic mean"),
              dict(refined, id="F28_REFINED", name="F28 / gentle refinement"),
              dict(sophy, id="SOPHY_SCALE", name="SOPHY / 1676.4 mm reference overlay")]
    if include_a0:
        models.extend([a0_model(), source_kit()])
    assets = []
    for model in models[:3]:
        path = output / (model["id"].lower()+".json")
        path.write_text(json.dumps(model, separators=(",", ":"), allow_nan=False)+"\n")
        stl = output / (model["id"].lower()+".stl")
        write_stl([p for p in model["parts"] if p["region"] != "envelope"], stl)
        assets.extend([path, stl])
    maquette_manifest = maquette(refined, output / "maquette")
    data = {"version": "1.0.0", "models": models, "reference": reference,
            "maquette": maquette_manifest,
            "maquette_files": {str(p.relative_to(output / "maquette")): p.read_text(encoding="utf-8")
                               for p in sorted((output / "maquette").rglob("*")) if p.is_file()},
            "project": "PHY / Mind Interfaces", "status": "demonstrable reference and shop-review studio"}
    template = (ROOT / "studio/src/index.html").read_text()
    bundle = (ROOT / "studio/app.bundle.js").read_text()
    # Escape script terminators, including any future user-edited source notes.
    encoded = json.dumps(data, separators=(",", ":"), allow_nan=False).replace("</", "<\\/")
    notices = (ROOT / "studio/THIRD_PARTY_NOTICES.txt").read_text()
    html = template.replace("__PHY_DATA__", encoded).replace("__PHY_APP__", bundle.replace("</script", "<\\/script"))
    html = "<!--\n" + notices + "\n-->\n" + html
    (output / "THIRD_PARTY_NOTICES.txt").write_text(notices)
    (output / "PHY-Studio.html").write_text(html, encoding="utf-8")
    measures = reference["statistics"]
    report = ["# PHY F28 / reference assembly", "", "Status: demonstrable whole-body form-study assembly; full-scale structural release remains open.", "",
              "92 measured female US Army participants aged exactly 28; arithmetic means, not medians or an attractiveness-selected subset.",
              "Population is not representative of all civilian/worldwide 28-year-old women. Marginal means do not describe one actual person.", "",
              "| Measurement | Mean mm | Sample SD mm |", "| --- | ---: | ---: |"]
    report += [f"| {key} | {row['mean_mm']:.3f} | {row['sd_mm']:.3f} |" for key, row in sorted(measures.items())]
    report += ["", "## Design mapping", "", "Age does not determine a unique skeletal geometry. The following are project design choices:",
               "- 220 mm chin–crown height; external head depth is not head height.",
               "- Shoulder centers 14 mm medial to each acromion; hip spacing is 0.68 × bicristal breadth and centers 35 mm above trochanterion.",
               "- Knee height uses lateral epicondyle height as a station proxy; ankle datum is 70 mm.",
               f"- The arm-chain station factor is {mean['design_datums']['arm_chain_closure_scale']:.6f}: allocate measured span among upper-arm/forearm/hand external ratios after subtracting shoulder-center spacing. This reconciles incompatible endpoint definitions; it is not measured bone length or an aesthetic adjustment.",
               "- 71 core individual bone proxies: 26 spine, 25 thorax, four shoulder girdle, two adult hips, six arm and eight leg bones.",
               "- 54 individual hand proxies add eight carpals, five metacarpals and 14 phalanges per side.",
               "- 52 individual foot proxies add seven tarsals, five metatarsals and 14 toe phalanges per side.",
               "- 22 individual skull proxies add eight cranial and 14 facial identities.",
               "- Six individual ear ossicles and one hyoid complete all 206 inventory identities; no grouped or unrepresented identities remain. See BONE_EQUIVALENCE.md.",
               "- The hand layout is a static display pose, not working opposition or measured bone geometry; see HAND_PROXY_LAYOUT.md.",
               "- The grounded foot layout is a static display, not measured bone geometry, functional arches or gait; see FOOT_PROXY_LAYOUT.md.",
               "- The skull layout is a static identity/distribution display, not measured surfaces, sutures or jaw mechanics; see SKULL_PROXY_LAYOUT.md.",
               "- Ear chains and the suspended hyoid are static display proxies; hearing, swallowing and soft-tissue mechanisms are unmodeled. See EAR_HYOID_PROXY_LAYOUT.md.",
               "- Bone-proxy layout is project-local provisional placement, not measured osteometry. Missing facial source records stay missing.",
               "- Couplings are hardware envelopes. Joint axes, bearing fits, retention and load paths are not resolved for the whole body.",
               "- Elliptical torso envelope uses marginal girths; interpolation and anatomical appearance are design, not scan data.", "",
               "Refinement: shoulders −1%, waist −3%, hips +2%. No measured source is overwritten, and no claim of objective attractiveness is made.", "",
               "The SOPHY scale overlay preserves H=span=1676.4 mm for comparison. It neither adopts new canon landmarks nor modifies canon 1.0.0.",
               "A0-R1 is the exact separate 35-instance CAD assembly. Its 317 mm station is never scaled or substituted into the whole-body form study.", "",
               "## First physical demonstration", "", "See maquette/BUILD.md, actual-size cut SVGs, BOM.csv and assembly-stencil.svg.",
               "The quarter-scale supported plywood maquette has a separate, preserved construction design. It does not miniaturize the core bone proxies or demonstrate skeletal distribution/structural capacity.", "",
               "## Reproduce", "", "`python bin/export_phy_studio.py` builds all geometry and the offline app; CadQuery 2.7.0 is required for A0 and the source kit.",
               "`--without-a0` builds the full-body reference and maquette with Python standard library only, using the checked-in app bundle.",
               "Viewer source: `npm ci --prefix studio && npm run build --prefix studio`.", "",
               "## Sources", "", reference["data_url"], reference["report_url"],
               "Pinned source CSV SHA-256: `"+reference["provenance"]["source_catalog"]+"` (catalog); projected subset hash in references/provenance.json."]
    (output / "REFERENCE.md").write_text("\n".join(report)+"\n")
    manifest = {"version": "1.0.0", "reference_sha256": reference["snapshot_sha256"],
                "models": [{"id": m["id"], "parts": len(m["parts"]), "triangles": sum(len(p["faces"]) for p in m["parts"]), "fabrication_released": m["fabrication_released"]} for m in models],
                "maquette_cut_parts": maquette_manifest["cut_parts"],
                "full_scale_structural_release": False,
                "files": {str(p.relative_to(output)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(output.rglob("*")) if p.is_file() and p.name not in ("manifest.json", "PHY-Studio-package.zip")}}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
    with zipfile.ZipFile(output / "PHY-Studio-package.zip", "w", compression=zipfile.ZIP_DEFLATED, compresslevel=9) as archive:
        for path in sorted(output.rglob("*")):
            if path.is_file() and path.suffix != ".zip":
                info = zipfile.ZipInfo(str(path.relative_to(output)), date_time=(1980, 1, 1, 0, 0, 0)); info.compress_type = zipfile.ZIP_DEFLATED
                archive.writestr(info, path.read_bytes())
    print(f"PHY Studio: {len(models)} models, {sum(len(m['parts']) for m in models)} viewable parts; {maquette_manifest['cut_parts']} maquette cuts.")
    return manifest


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "studio/dist")
    parser.add_argument("--without-a0", action="store_true")
    parser.add_argument("--sync-review", action="store_true", help="copy generated offline HTML and bone reports to checked-in review locations")
    args = parser.parse_args()
    export(args.output, not args.without_a0)
    if args.sync_review:
        review = ROOT / "PROJECTS/PHY_F28/reports"
        review.mkdir(parents=True, exist_ok=True)
        for name in ("BONE_EQUIVALENCE.json", "BONE_EQUIVALENCE.md", "CORE_PROXY_LAYOUT.md", "HAND_PROXY_LAYOUT.md", "FOOT_PROXY_LAYOUT.md", "SKULL_PROXY_LAYOUT.md", "EAR_HYOID_PROXY_LAYOUT.md"):
            shutil.copyfile(args.output / name, review / name)
        shutil.copyfile(args.output / "PHY-Studio.html", ROOT / "studio/PHY-Studio.html")
