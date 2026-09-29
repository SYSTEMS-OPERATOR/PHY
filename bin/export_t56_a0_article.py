#!/usr/bin/env python3
"""Export the bounded T56 A0-R1 shoulder article shop-review packet.

CadQuery 2.7.0 is required. Outputs remain unreleased and unqualified.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "PROJECTS/T56_CARBON"
DESIGN_PATH = PROJECT / "requirements/a0_shoulder_mechanism.json"
DEFAULT_OUTPUT = PROJECT / "fabrication/a0_packet"


def load_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def load_design():
    return load_json(DESIGN_PATH)


def fabrication(design=None):
    return (design or load_design())["fabrication_geometry"]


def bom_rows(design=None):
    f = fabrication(design)
    c, d, q = f["cartridge"], f["dummy_member"], f["four_bar"]
    return [
        ("A0-101", 1, "fixture base plate", "A36 steel plate", f"{f['fixture']['base_size_xyz']}; A0-D001-R1"),
        ("A0-102", 2, "parallel four-bar rocker", "6061-T6 plate", f"{q['rocker_size_xyz']}; {q['rocker_centers']} centers"),
        ("A0-103", 1, "moving carriage plate", "6061-T6 plate", f"{q['carriage_size_xyz']}"),
        ("A0-104", 2, "pitch yoke side plate", "6061-T6 plate", f"{c['yoke_plate_size_xzy']}; Ø35 H7 seat"),
        ("A0-105", 1, "split-clamp offset output hub", "6061-T6 billet", f"{c['hub_size_xyz']}; {c['tube_socket_across_flats']} socket"),
        ("A0-106", 1, "keyed index sector", "6061-T6 plate", f"Ø{c['sector_od']} x {c['sector_thickness']}; 15 degree index"),
        ("A0-107", 4, "replaceable stop pad", "acetal", f"{c['stop_pad_size_xyz']}"),
        ("A0-108", 1, "dummy humeral member", "6061-T6 square tube", f"{d['outer']} square x {d['wall']} wall x {d['cut_length']}"),
        ("A0-109", 2, "root pivot standoff", "6061-T6 billet", f"{f['fixture']['standoff_size_xyz']}; axis Z={f['root_axis_height']}"),
        ("A0-110", 4, "hard-stop bracket", "6061-T6 billet", f"{c['stop_bracket_size_xyz']}; final contact blue-check open"),
        ("A0-201", 1, "pitch shaft", "4140 prehard ground bar", f"Ø17 h6 x {c['shaft_length']}; 5 x 5 keyway"),
        ("A0-202", 1, "parallel key", "hardened steel, DIN 6885 form A", "5 x 5 x 30"),
        ("A0-203", 2, "shaft end retainer", "4140 or class 10.9 steel", f"Ø{c['end_retainer_od_width'][0]} x {c['end_retainer_od_width'][1]}; detail review open"),
        ("A0-301", 2, "deep-groove bearing", c["bearing_type"], "17 x 35 x 10; lot and corner geometry verified before machining"),
        ("A0-302", 4, "pivot shoulder bolt", "ISO 898-1 class 10.9", "M10 x 1.5; 10 shoulder"),
        ("A0-303", 4, "pivot locknut", "all-metal prevailing torque", "M10 x 1.5; single-use"),
        ("A0-304", 2, "spring indexing plunger", "steel, pull-ring type", "8 pin; body/thread and nose geometry supplier-open"),
        ("A0-305", 1, "carriage clamp screw", "ISO 898-1 class 10.9", "M8 x 1.25 with captive hand knob"),
        ("A0-306", 2, "hub pinch bolt", "ISO 898-1 class 10.9", "M6 x 1.0; length after first-article stack"),
        ("A0-307", 1, "secondary retention tether", "steel wire rope", "6 mm; rated >=1 kN; end fittings supplier-open"),
        ("A0-308", 4, "bench mounting fastener", "ISO 898-1 class 10.9", "M10 x 1.5; length to reviewed bench"),
        ("A0-309", 1, "passive friction stack", "dry friction + Belleville washers", "breakaway >=7 N m; material/correlation supplier-open"),
        ("A0-310", 2, "bearing retaining cap", "6061-T6", "captures protruding 6003 ring; detail review open"),
        ("A0-401", 1, "galvanic isolation kit", "PET plus zinc-rich primer", "0.5 PET; seal aluminum/steel interfaces"),
    ]


def build_parts(design=None):
    import cadquery as cq

    design = design or load_design()
    f = fabrication(design)
    fixture, q, c, d = f["fixture"], f["four_bar"], f["cartridge"], f["dummy_member"]

    sx, sy, sz = fixture["base_size_xyz"]
    base = cq.Workplane("XY").box(sx, sy, sz).translate((0, 0, -sz / 2))
    base = base.faces(">Z").workplane().pushPoints([(-120, -90), (-120, 90), (120, -90), (120, 90)]).hole(fixture["bench_hole_diameter"])
    base = base.faces(">Z").workplane().pushPoints([(0, -60), (0, 40)]).hole(fixture["root_hole_diameter"])
    base = base.faces(">Z").workplane().pushPoints([(130, 0)]).hole(fixture["tooling_hole_diameter"])

    ss = fixture["standoff_size_xyz"]
    standoff = cq.Workplane("XY").box(*ss).translate((0, 0, ss[2] / 2))
    standoff = standoff.faces(">Z").workplane().hole(fixture["root_hole_diameter"])

    rs = q["rocker_size_xyz"]
    rocker = cq.Workplane("XY").box(*rs)
    rocker = rocker.faces(">Z").workplane().pushPoints([(-q["rocker_centers"] / 2, 0), (q["rocker_centers"] / 2, 0)]).hole(q["pivot_bore_diameter"])
    cs = q["carriage_size_xyz"]
    carriage = cq.Workplane("XY").box(*cs)
    carriage = carriage.faces(">Z").workplane().pushPoints([(0, -q["carriage_pivot_centers"] / 2), (0, q["carriage_pivot_centers"] / 2)]).hole(q["pivot_bore_diameter"])
    carriage = carriage.faces(">Z").workplane().pushPoints([(-20, -30), (-20, 30)]).hole(6.6)

    yx, yz, yt = c["yoke_plate_size_xzy"]
    yoke = cq.Workplane("XZ").rect(yx, yz).extrude(yt / 2, both=True)
    yoke = yoke.faces(">Y").workplane().hole(c["bearing_housing_diameter"])
    yoke = yoke.faces(">Y").workplane().pushPoints([(-20, -27.5), (20, -27.5)]).hole(6.6)

    hx, hy, hz = c["hub_size_xyz"]
    hub = cq.Workplane("XY").box(hx, hy, hz).translate((c["hub_center_x_from_S"], 0, 0))
    shaft_void = cq.Solid.makeCylinder(c["shaft_diameter"] / 2, hy + 2, cq.Vector(0, -hy / 2 - 1, 0), cq.Vector(0, 1, 0))
    hub = hub.cut(shaft_void)
    socket_length = abs(c["tube_socket_seat_x_from_S"] - c["tube_socket_start_x_from_S"]) + 1
    socket_center = (c["tube_socket_seat_x_from_S"] + c["tube_socket_start_x_from_S"]) / 2
    socket = cq.Workplane("XY").box(socket_length, c["tube_socket_across_flats"], c["tube_socket_across_flats"]).translate((socket_center, 0, 0))
    hub = hub.cut(socket)
    hub = hub.cut(cq.Workplane("XY").box(socket_length + 4, 1.5, hz / 2).translate((socket_center, 0, hz / 4)))
    hub = hub.cut(cq.Workplane("XY").box(5, hy + 2, 5).translate((0, 0, c["shaft_diameter"] / 2)))
    for xpos in c["hub_pinch_bolt_x_from_S"]:
        hub = hub.cut(cq.Solid.makeCylinder(c["hub_pinch_bolt_diameter"] / 2, hz + 2, cq.Vector(xpos, 0, -hz / 2 - 1), cq.Vector(0, 0, 1)))

    sector = cq.Workplane("XZ").circle(c["sector_od"] / 2).circle(c["sector_bore_diameter"] / 2).extrude(c["sector_thickness"] / 2, both=True)
    sector = sector.cut(cq.Workplane("XZ").rect(5, 5).extrude(c["sector_thickness"], both=True).translate((0, 0, c["shaft_diameter"] / 2)))
    angles = range(-30, 91, c["index_increment_deg"])
    holes = [(c["index_hole_radius"] * math.cos(math.radians(a)), c["index_hole_radius"] * math.sin(math.radians(a))) for a in angles]
    sector = sector.faces(">Y").workplane().pushPoints(holes).hole(c["index_hole_diameter"])

    outer, wall, length = d["outer"], d["wall"], d["cut_length"]
    dummy = cq.Workplane("YZ").rect(outer, outer).extrude(length)
    dummy = dummy.cut(cq.Workplane("YZ").rect(outer - 2 * wall, outer - 2 * wall).extrude(length + 2).translate((-1, 0, 0)))
    shaft = cq.Workplane("XZ").circle(c["shaft_diameter"] / 2).extrude(c["shaft_length"] / 2, both=True)
    shaft = shaft.cut(cq.Workplane("XY").box(5, 30, 2.5).translate((0, 0, c["shaft_diameter"] / 2 - 1.25)))
    kw, kh, kl = c["shaft_key_width_height_length"]
    key = cq.Workplane("XY").box(kw, kl, kh)
    bearing = cq.Workplane("XZ").circle(c["bearing_id_od_width"][1] / 2).circle(c["bearing_id_od_width"][0] / 2).extrude(c["bearing_id_od_width"][2] / 2, both=True)
    friction = cq.Workplane("XZ").circle(c["friction_stack_envelope_od_width"][0] / 2).circle(c["shaft_diameter"] / 2).extrude(c["friction_stack_envelope_od_width"][1] / 2, both=True)
    retainer = cq.Workplane("XZ").circle(c["end_retainer_od_width"][0] / 2).circle(c["shaft_diameter"] / 2).extrude(c["end_retainer_od_width"][1] / 2, both=True)
    stop = cq.Workplane("XY").box(*c["stop_pad_size_xyz"])
    bracket = cq.Workplane("XY").box(*c["stop_bracket_size_xyz"])
    plunger = cq.Workplane("XZ").circle(c["plunger_envelope_body_pin_length"][0] / 2).extrude(c["plunger_envelope_body_pin_length"][2] / 2, both=True)
    return {
        "A0-101_fixture_base": base, "A0-102_rocker": rocker, "A0-103_carriage": carriage,
        "A0-104_yoke_side": yoke, "A0-105_output_hub": hub, "A0-106_index_sector": sector,
        "A0-107_stop_pad": stop, "A0-108_dummy_member": dummy, "A0-109_root_standoff": standoff,
        "A0-110_stop_bracket": bracket, "A0-201_pitch_shaft": shaft, "A0-202_key": key,
        "A0-203_end_retainer": retainer, "A0-301_bearing_envelope": bearing,
        "A0-304_plunger_envelope": plunger, "A0-309_friction_stack_envelope": friction,
    }


def located_instances(parts, design=None):
    """Return neutral-pose instances as (instance id, part id, shape)."""
    design = design or load_design()
    f = fabrication(design); c = f["cartridge"]
    stage = design["mechanism"]["scapular_stage"]
    s = design["mechanism"]["humeral_stage"]["joint_center_mm"]
    p, a = stage["posterior_root_mm"], stage["anterior_root_mm"]
    b, cc = stage["posterior_carriage_pivot_mm"], stage["anterior_carriage_pivot_mm"]
    mid = lambda u, v: tuple((u[i] + v[i]) / 2 for i in range(3))
    inst = []
    add = lambda iid, pid, shape: inst.append((iid, pid, shape))
    add("fixture", "A0-101_fixture_base", parts["A0-101_fixture_base"].translate((-130, 0, 0)))
    for name, point in (("P", p), ("A", a)):
        add(f"root_standoff_{name}", "A0-109_root_standoff", parts["A0-109_root_standoff"].translate((point[0], point[1], 0)))
    add("rocker_PB", "A0-102_rocker", parts["A0-102_rocker"].translate(mid(p, b)))
    add("rocker_AC", "A0-102_rocker", parts["A0-102_rocker"].translate(mid(a, cc)))
    add("carriage", "A0-103_carriage", parts["A0-103_carriage"].translate(mid(b, cc)))
    for index, ypos in enumerate(c["yoke_plate_center_y"]):
        add(f"yoke_{index+1}", "A0-104_yoke_side", parts["A0-104_yoke_side"].translate((s[0], ypos, s[2])))
        add(f"bearing_{index+1}", "A0-301_bearing_envelope", parts["A0-301_bearing_envelope"].translate((s[0], ypos, s[2])))
    add("hub", "A0-105_output_hub", parts["A0-105_output_hub"].translate(s))
    add("shaft", "A0-201_pitch_shaft", parts["A0-201_pitch_shaft"].translate(s))
    add("key", "A0-202_key", parts["A0-202_key"].translate((s[0], s[1], s[2] + c["shaft_diameter"] / 2 + 1.25)))
    add("sector", "A0-106_index_sector", parts["A0-106_index_sector"].translate((s[0], c["sector_center_y"], s[2])))
    add("friction_stack", "A0-309_friction_stack_envelope", parts["A0-309_friction_stack_envelope"].translate((s[0], c["friction_stack_center_y"], s[2])))
    shaft_half = c["shaft_length"] / 2
    for index, ypos in enumerate((s[1] - shaft_half + 2, s[1] + shaft_half - 2)):
        add(f"retainer_{index+1}", "A0-203_end_retainer", parts["A0-203_end_retainer"].translate((s[0], ypos, s[2])))
    cut_start = s[0] + f["dummy_member"]["cut_start_x_from_S"]
    member = parts["A0-108_dummy_member"].rotate((0, 0, 0), (0, 0, 1), 180).translate((cut_start, s[1], s[2]))
    add("dummy_member", "A0-108_dummy_member", member)
    for index, zoff in enumerate((-45, 45)):
        add(f"pitch_stop_pad_{index+1}", "A0-107_stop_pad", parts["A0-107_stop_pad"].translate((s[0] + 35, s[1], s[2] + zoff)))
        add(f"pitch_stop_bracket_{index+1}", "A0-110_stop_bracket", parts["A0-110_stop_bracket"].translate((s[0] + 52, s[1], s[2] + zoff)))
    add("index_plunger", "A0-304_plunger_envelope", parts["A0-304_plunger_envelope"].translate((s[0] + 35, c["sector_center_y"], s[2])))
    return inst


def motion_report(design=None):
    design = design or load_design(); f = fabrication(design)
    stage = design["mechanism"]["scapular_stage"]
    s0 = design["mechanism"]["humeral_stage"]["joint_center_mm"]
    length = design["dummy_member"]["effective_length_mm"]
    exclusions = design["collision_and_service_envelopes"]
    samples, collision_count = [], 0
    for scap in f["assembly"]["sampled_scapular_angles_deg"]:
        sr = math.radians(scap)
        s = [-130 - stage["rocker_center_distance_mm"] * math.cos(sr) - 20,
             -10 + stage["rocker_center_distance_mm"] * math.sin(sr), s0[2]]
        for pitch in f["assembly"]["sampled_pitch_angles_deg"]:
            pr = math.radians(pitch)
            e = [s[0] - length * math.cos(pr), s[1], s[2] + length * math.sin(pr)]
            hit = []
            for key in ("neck_exclusion_aabb_mm", "representative_ribcage_exclusion_aabb_mm"):
                box = exclusions[key]; low, high = box["min_mm"], box["max_mm"]
                for n in range(65):
                    t = n / 64
                    point = [s[i] + t * (e[i] - s[i]) for i in range(3)]
                    if all(low[i] <= point[i] <= high[i] for i in range(3)):
                        hit.append(key); break
            collision_count += bool(hit)
            samples.append({"scapular_deg": scap, "pitch_deg": pitch, "S_mm": [round(v, 6) for v in s], "E_mm": [round(v, 6) for v in e], "registered_exclusion_hits": sorted(set(hit))})
    return {
        "method": "discrete centerline samples against registered AABBs",
        "continuous_clearance_proof": False, "sample_count": len(samples),
        "sampled_collision_count": collision_count,
        "result": "pass_at_samples_only" if collision_count == 0 else "blocked_collision_at_sample",
        "intentional_contacts": f["assembly"]["intentional_contact_classes"],
        "limitations": ["not a swept-solid or continuous collision proof", "purchased hardware envelopes are provisional", "guard and external bench are not modeled", "tool clearance requires independent drawing and dry-assembly review"],
        "samples": samples,
    }


def normalize_step(path):
    content = path.read_text(encoding="utf-8")
    content = re.sub(r"(FILE_NAME\('Open CASCADE Shape Model',')[^']+(')", r"\g<1>1970-01-01T00:00:00\2", content)
    path.write_text("\n".join(line.rstrip() for line in content.splitlines()) + "\n", encoding="utf-8")


def svg_sheet(number, title, rows):
    body = [f'<text x="40" y="50" font-size="25">{number} — {title}</text>', '<text x="40" y="80" font-size="14">A0-R1 SHOP REVIEW CANDIDATE — NOT FABRICATION RELEASED — ALL DIMENSIONS mm — DO NOT SCALE</text>', '<line x1="40" y1="105" x2="1360" y2="105" stroke="#172230" stroke-width="2"/>']
    for index, (name, value) in enumerate(rows):
        y = 145 + index * 29
        body.append(f'<text x="60" y="{y}" font-size="15">{name}</text><text x="620" y="{y}" font-size="15">{value}</text>')
    return '<svg xmlns="http://www.w3.org/2000/svg" width="1400" height="900" viewBox="0 0 1400 900"><rect width="1400" height="900" fill="white"/><g font-family="monospace" fill="#172230"><rect x="20" y="20" width="1360" height="860" fill="none" stroke="#172230" stroke-width="2"/>' + "".join(body) + '</g></svg>\n'


def drawings(design):
    f = fabrication(design); c = f["cartridge"]; q = f["four_bar"]
    common = [("FRAME", design["frame_id"]), ("CANON EFFECT", "none"), ("STATUS", "independent review required; physical evidence open")]
    return {
        "A0-D001-R1-fixture-datum.svg": svg_sheet("A0-D001-R1", "FIXTURE DATUM AND ROOT INTERFACE", common + [("BASE", "A36 300 x 240 x 8; bounds X -280..20, Y +/-120, Z -8..0"), ("DATUM", "Ø6 H7 tooling axis at (0,0), plate top Z=0"), ("BENCH HOLES", "Ø11 at (-250,+/-90), (-10,+/-90)"), ("ROOT AXES", "P=(-130,-60,35), A=(-130,40,35), position +/-0.25"), ("STANDOFFS", "2 x 25 x 25 x 32; root axis Z=35"), ("ISOLATION", "0.5 PET + zinc-rich primer at Al/A36 contacts")]),
        "A0-D002-R1-mechanism-assembly.svg": svg_sheet("A0-D002-R1", "FOUR-BAR AND PITCH ASSEMBLY", common + [("FOUR-BAR", f"rockers {q['rocker_centers']} centers; coupler {q['carriage_pivot_centers']} centers; +/-10 deg"), ("OUTPUT S", "(-205,-10,70) neutral; sampled trajectory in motion_clearance_report.json"), ("PITCH", "axis +Y; command -30..+90; independent stops -32/+92"), ("YOKES", "2 x 70 x 70 x 8 at Y=-40,+20; 6003 seats Ø35 H7"), ("PASSIVE", ">=7 N m dry friction + 8 mm index at 15 deg"), ("SECONDARY", ">=1 kN tether; endpoints provisional pending fitting selection")]),
        "A0-D003-R1-dummy-member.svg": svg_sheet("A0-D003-R1", "DUMMY MEMBER AND STATION STACK", common + [("SECTION", "6061-T6 tube 25.4 square x 3.175 wall"), ("CUT", "327 +/-0.5; cut_S=(-215,-10,70), cut_E=(-542,-10,70)"), ("STATIONS", "S=(-205,-10,70), E=(-522,-10,70), effective 317 +/-1.0"), ("SEATS", "shoulder offset 40/insertion 30; elbow offset 25/insertion 45"), ("CENTERLINE", "project-local mechanics only; not anatomical canon")]),
        "A0-D004-R1-cartridge-interfaces.svg": svg_sheet("A0-D004-R1", "PITCH CARTRIDGE INTERFACES", common + [("SHAFT", f"4140 Ø17 h6 x {c['shaft_length']}; keyed 5 x 5 x 30; captured ends"), ("BEARINGS", "2 x 6003-2RS, 17 x 35 x 10; yoke bores Ø35 H7"), ("HUB", "offset split clamp; socket 25.50 +0.15/-0.00; seat X=-40 from S"), ("PINCH", "2 x M6-10.9 at X=-20,-32 from S; provisional 12 N m"), ("SECTOR", "Ø90 x 6, keyed Ø17 interface, Ø8.2 holes each 15 deg"), ("SUPPLIER HOLD", "bearing reliefs, plunger thread/nose and friction stack correlation open")]),
        "A0-D005-R1-assembly-register.svg": svg_sheet("A0-D005-R1", "LOCATED ASSEMBLY AND QUANTITY REGISTER", common + [("ASSEMBLY", "assembly/A0-R1-neutral-assembly.step and assembly_instances.json"), ("PRIMARY", "base 1; standoffs 2; rockers 2; carriage 1; yokes 2; hub 1; member 1"), ("CARTRIDGE", "shaft 1; key 1; bearings 2; sector 1; friction stack 1; end retainers 2"), ("SAFETY", "stop pads/brackets shown as provisional contact envelopes"), ("CONTACT", "intentional-contact classes declared in mechanism JSON"), ("LIMIT", "located neutral model is review geometry, not proof of tolerance or capacity")]),
        "A0-D006-R1-exploded-order.svg": svg_sheet("A0-D006-R1", "EXPLODED ORDER AND HOLD POINTS", common + [("1", "base -> isolation -> root standoffs -> four-bar -> carriage"), ("2", "yoke plates -> bearing seats/caps -> shaft/key -> hub"), ("3", "sector -> friction stack -> end retainers -> plunger -> hard stops"), ("4", "dummy member to 30 mm shoulder insertion mark -> tether -> guards"), ("HOLD", "supplier dimensions before mating features; independent load-path review before stock"), ("EVIDENCE", "inspection and bench records remain deliberately blank")]),
    }


def export(output):
    import cadquery as cq

    design = load_design(); f = fabrication(design)
    cad_dir, drawing_dir, assembly_dir = output / "cad", output / "drawings", output / "assembly"
    for folder in (cad_dir / "step", cad_dir / "stl", drawing_dir, assembly_dir):
        folder.mkdir(parents=True, exist_ok=True)
    parts = build_parts(design); checks = []
    for name, workplane in parts.items():
        shape = workplane.val()
        if not shape.isValid() or len(shape.Solids()) != 1 or shape.Volume() <= 0:
            raise ValueError(f"invalid solid: {name}")
        step, stl = cad_dir / "step" / f"{name}.step", cad_dir / "stl" / f"{name}.stl"
        cq.exporters.export(shape, str(step)); cq.exporters.export(shape, str(stl), tolerance=0.05, angularTolerance=0.1)
        normalize_step(step)
        reloaded = cq.importers.importStep(str(step)).val()
        if not reloaded.isValid() or not math.isclose(reloaded.Volume(), shape.Volume(), rel_tol=1e-6):
            raise ValueError(f"STEP round trip failed: {name}")
        box = shape.BoundingBox()
        checks.append({"part": name, "valid_solid": True, "step_round_trip": "pass", "bounds_mm": [box.xlen, box.ylen, box.zlen], "volume_mm3": shape.Volume()})

    instances = located_instances(parts, design)
    compound = cq.Compound.makeCompound([item[2].val() for item in instances])
    assembly_step = assembly_dir / "A0-R1-neutral-assembly.step"
    cq.exporters.export(compound, str(assembly_step)); normalize_step(assembly_step)
    if not cq.importers.importStep(str(assembly_step)).val().isValid():
        raise ValueError("assembly STEP round trip failed")
    instance_rows = [{"instance_id": iid, "part_id": pid, "valid": shape.val().isValid()} for iid, pid, shape in instances]
    (assembly_dir / "assembly_instances.json").write_text(json.dumps({"frame_id": design["frame_id"], "pose_id": design["pose_id"], "instances": instance_rows}, indent=2) + "\n", encoding="utf-8")
    motion = motion_report(design)
    (assembly_dir / "motion_clearance_report.json").write_text(json.dumps(motion, indent=2) + "\n", encoding="utf-8")

    drawing_set = drawings(design)
    for name, content in drawing_set.items():
        (drawing_dir / name).write_text(content, encoding="utf-8")
    with (output / "BOM.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle, lineterminator="\n")
        writer.writerow(["item_id", "qty", "description", "material_or_standard", "size_or_procurement_note"]); writer.writerows(bom_rows(design))
    manifest = {
        "packet_id": design["packet_id"], "revision": f["revision"], "status": "independent_shop_review_candidate_not_fabrication_released",
        "fabrication_released": False, "physical_evidence_complete": False, "canon_effect": design["canon_effect"], "units": "mm",
        "source": str(DESIGN_PATH.relative_to(ROOT)), "source_sha256": hashlib.sha256(DESIGN_PATH.read_bytes()).hexdigest(),
        "parts": checks, "assembly_step": str(assembly_step.relative_to(output)), "assembly_instance_count": len(instances),
        "motion_clearance": {k: motion[k] for k in ("method", "continuous_clearance_proof", "sample_count", "sampled_collision_count", "result")},
        "drawing_files": sorted(drawing_set), "bom_items": len(bom_rows(design)), "supplier_dependent_open_items": f["supplier_dependent_open_items"],
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"Exported A0-R1: {len(parts)} parts, {len(instances)} instances, {len(drawing_set)} drawings, {len(bom_rows(design))} BOM lines")
    return manifest


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    export(parser.parse_args().output)
