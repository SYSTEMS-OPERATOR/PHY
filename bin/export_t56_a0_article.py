#!/usr/bin/env python3
"""Export the bounded T56 A0 shoulder article CAD and review drawings.

Requires CadQuery 2.7.0.  The outputs are a shop-review candidate, not a
fabrication release or physical qualification record.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DESIGN_PATH = ROOT / "PROJECTS/T56_CARBON/requirements/a0_shoulder_mechanism.json"
DEFAULT_OUTPUT = ROOT / "PROJECTS/T56_CARBON/fabrication/a0_packet"


def load_design():
    return json.loads(DESIGN_PATH.read_text(encoding="utf-8"))


def bom_rows():
    return [
        ("A0-101", 1, "fixture base plate", "A36 steel plate", "300 x 240 x 8 mm; A0-D001"),
        ("A0-102", 2, "parallel four-bar rocker", "6061-T6 plate", "70 x 25 x 6 mm; 55 mm centers"),
        ("A0-103", 1, "moving carriage plate", "6061-T6 plate", "70 x 120 x 8 mm"),
        ("A0-104", 2, "pitch yoke side plate", "6061-T6 plate", "60 x 70 x 8 mm; bearing bore 28 H7"),
        ("A0-105", 1, "split-clamp output hub", "6061-T6 billet", "55 x 40 x 55 mm; 25.50 mm socket"),
        ("A0-106", 1, "index sector", "6061-T6 plate", "90 OD x 6 mm; 15 degree index"),
        ("A0-107", 4, "replaceable stop pad", "acetal", "20 x 15 x 10 mm"),
        ("A0-108", 1, "dummy humeral member", "6061-T6 square tube", "25.4 x 25.4 x 3.175 wall x 327 mm"),
        ("A0-201", 1, "pitch shaft", "4140 prehard ground bar", "12 h6 x 70 mm; ends retained"),
        ("A0-301", 2, "deep-groove bearing", "6001-2RS", "12 x 28 x 8 mm; supplier lot recorded"),
        ("A0-302", 4, "pivot shoulder bolt", "ISO 898-1 class 10.9", "M10 x 1.5; 10 mm shoulder"),
        ("A0-303", 4, "pivot locknut", "all-metal prevailing torque", "M10 x 1.5; single-use"),
        ("A0-304", 2, "spring indexing plunger", "steel, pull-ring type", "M16 x 1.5 body; 8 mm pin; vendor cross-check open"),
        ("A0-305", 1, "carriage clamp screw", "ISO 898-1 class 10.9", "M8 x 1.25 with captive hand knob"),
        ("A0-306", 2, "hub pinch bolt", "ISO 898-1 class 10.9", "M6 x 1.0 x 35 mm"),
        ("A0-307", 1, "secondary retention tether", "steel wire rope", "6 mm; rated >= 1 kN; thimbles and swages"),
        ("A0-308", 4, "bench mounting fastener", "ISO 898-1 class 10.9", "M10 x 1.5; length to bench fixture"),
        ("A0-401", 1, "galvanic isolation kit", "PET plus zinc-rich primer", "0.5 mm PET; seal all aluminum/steel interfaces"),
    ]


def build_parts():
    import cadquery as cq

    base = (
        cq.Workplane("XY").box(300, 240, 8)
        .faces(">Z").workplane()
        .pushPoints([(-120, -90), (-120, 90), (120, -90), (120, 90)]).hole(11)
        .faces(">Z").workplane().pushPoints([(0, -60), (0, 40)]).hole(10.5)
        .faces(">Z").workplane().pushPoints([(130, 0)]).hole(6)
    )
    rocker = (
        cq.Workplane("XY").box(70, 25, 6)
        .faces(">Z").workplane().pushPoints([(-27.5, 0), (27.5, 0)]).hole(10.2)
    )
    carriage = (
        cq.Workplane("XY").box(70, 120, 8)
        .faces(">Z").workplane().pushPoints([(10, -50), (10, 50)]).hole(10.2)
        .faces(">Z").workplane().pushPoints([(-15, -35), (-15, 35)]).hole(6.6)
    )
    yoke = (
        cq.Workplane("XZ").rect(60, 70).extrude(8, both=True)
        .faces(">Y").workplane().hole(28)
        .faces(">Y").workplane().pushPoints([(-15, -27.5), (15, -27.5)]).hole(6.6)
    )
    hub = cq.Workplane("XY").box(55, 40, 55)
    hub = hub.cut(cq.Solid.makeCylinder(6, 42, cq.Vector(0, -21, 0), cq.Vector(0, 1, 0)))
    hub = hub.cut(cq.Workplane("YZ").rect(25.5, 25.5).extrude(31).translate((-28, 0, 0)))
    hub = hub.cut(cq.Workplane("XY").box(32, 1.5, 30).translate((-12, 0, 20)))
    sector = cq.Workplane("XZ").circle(45).circle(6).extrude(6, both=True)
    holes = [(35 * math.cos(math.radians(a)), 35 * math.sin(math.radians(a))) for a in range(-30, 91, 15)]
    sector = sector.faces(">Y").workplane().pushPoints(holes).hole(8.2)
    dummy = cq.Workplane("YZ").rect(25.4, 25.4).extrude(327)
    dummy = dummy.cut(cq.Workplane("YZ").rect(19.05, 19.05).extrude(329).translate((-1, 0, 0)))
    shaft = cq.Workplane("XZ").circle(6).extrude(70)
    stop = cq.Workplane("XY").box(20, 15, 10)
    return {
        "A0-101_fixture_base": base,
        "A0-102_rocker": rocker,
        "A0-103_carriage": carriage,
        "A0-104_yoke_side": yoke,
        "A0-105_output_hub": hub,
        "A0-106_index_sector": sector,
        "A0-107_stop_pad": stop,
        "A0-108_dummy_member": dummy,
        "A0-201_pitch_shaft": shaft,
    }


def drawing(title, subtitle, body):
    return f'''<svg xmlns="http://www.w3.org/2000/svg" width="1400" height="900" viewBox="0 0 1400 900">
<rect width="1400" height="900" fill="white"/><g font-family="monospace" fill="#172230">
<text x="40" y="48" font-size="25">{title}</text><text x="40" y="78" font-size="15">{subtitle}</text>
<rect x="25" y="20" width="1350" height="850" fill="none" stroke="#172230" stroke-width="2"/>{body}
<text x="40" y="842" font-size="14">A0 SHOP REVIEW CANDIDATE — NOT FABRICATION RELEASED — ALL DIMENSIONS mm — DO NOT SCALE</text>
</g></svg>\n'''


def drawings(design):
    fixture = drawing(
        "A0-D001-R0 — FIXTURE DATUM AND ROOT INTERFACE",
        "FRAME_T56_THORAX: +X right, +Y anterior, +Z superior; origin at Ø6 H7 tooling-hole axis, plate top Z=0",
        '''<rect x="170" y="160" width="750" height="600" fill="#eef2f3" stroke="#172230" stroke-width="3"/>
<line x1="845" y1="440" x2="1060" y2="440" stroke="#bf4b3f" stroke-width="3"/><text x="1070" y="446" font-size="17">+X</text>
<line x1="845" y1="440" x2="845" y2="235" stroke="#2f7c5d" stroke-width="3"/><text x="830" y="220" font-size="17">+Y</text>
<circle cx="845" cy="440" r="9" fill="none" stroke="#172230" stroke-width="3"/><text x="870" y="425" font-size="15">ORIGIN Ø6 H7</text>
<g fill="none" stroke="#172230" stroke-width="3"><circle cx="245" cy="665" r="14"/><circle cx="245" cy="215" r="14"/><circle cx="845" cy="665" r="14"/><circle cx="845" cy="215" r="14"/></g>
<g fill="#156f88"><circle cx="520" cy="590" r="12"/><circle cx="520" cy="340" r="12"/></g>
<text x="955" y="175" font-size="16">PLATE: A36 300 × 240 × 8</text><text x="955" y="205" font-size="16">BENCH: 4× Ø11 THRU</text>
<text x="955" y="235" font-size="16">ROOTS: 2× Ø10.5 THRU</text><text x="955" y="265" font-size="16">ROOT HEIGHT: Z=35 ±0.25</text>
<text x="955" y="310" font-size="15">P = (-130,-60,35) ±0.25</text><text x="955" y="340" font-size="15">A = (-130, 40,35) ±0.25</text>
<text x="955" y="390" font-size="15">BENCH HOLES:</text><text x="955" y="420" font-size="14">(-250,±90), (-10,±90)</text>
<text x="955" y="470" font-size="15">DATUM: TRUE POSITION 0.25</text><text x="955" y="500" font-size="15">FLATNESS: 0.30 OVER PLATE</text>
<text x="955" y="550" font-size="14">PET barrier + zinc-rich primer</text><text x="955" y="575" font-size="14">at aluminum/steel interfaces.</text>''')
    mechanism = drawing(
        "A0-D002-R0 — FOUR-BAR AND PITCH CARTRIDGE",
        "Neutral top view; P-A fixed, B-C moving; shoulder output S shown at A0_NEUTRAL_LOCKED",
        '''<g fill="none" stroke="#172230" stroke-width="8"><line x1="330" y1="620" x2="650" y2="620"/><line x1="330" y1="300" x2="650" y2="300"/><line x1="650" y1="300" x2="650" y2="620"/></g>
<g fill="#156f88"><circle cx="330" cy="620" r="13"/><circle cx="330" cy="300" r="13"/><circle cx="650" cy="620" r="13"/><circle cx="650" cy="300" r="13"/><circle cx="770" cy="460" r="16"/></g>
<text x="300" y="655" font-size="18">P</text><text x="300" y="285" font-size="18">A</text><text x="665" y="650" font-size="18">B</text><text x="665" y="285" font-size="18">C</text><text x="790" y="465" font-size="18">S</text>
<line x1="330" y1="700" x2="650" y2="700" stroke="#bf4b3f" stroke-width="2"/><text x="455" y="730" font-size="18">55 ±0.25</text>
<line x1="700" y1="300" x2="700" y2="620" stroke="#bf4b3f" stroke-width="2"/><text x="715" y="470" font-size="18">100 ±0.25</text>
<path d="M 760 425 Q 790 460 760 495" fill="none" stroke="#bf4b3f" stroke-width="3"/><text x="820" y="420" font-size="15">scapular stops ±10°</text>
<text x="930" y="270" font-size="16">S = (-205,-10,70) ±0.5</text><text x="930" y="305" font-size="16">pitch axis = (0,1,0)</text>
<text x="930" y="340" font-size="16">command: -30° to +90°</text><text x="930" y="375" font-size="16">hard stops: -32° / +92°</text>
<text x="930" y="425" font-size="15">RETENTION:</text><text x="930" y="455" font-size="14">8 mm spring index pin, 15° holes</text>
<text x="930" y="480" font-size="14">6 mm secondary steel tether ≥1 kN</text><text x="930" y="520" font-size="14">software is never the sole stop</text>
<text x="930" y="580" font-size="14">Pivot pattern true position: Ø0.30</text><text x="930" y="605" font-size="14">Axis parallelism: 0.25°</text>''')
    member = drawing(
        "A0-D003-R0 — DUMMY HUMERAL MEMBER",
        "Straight removable A0 load member; project-local mechanics, not an anatomical bone or canon landmark",
        '''<rect x="180" y="350" width="930" height="72" fill="#eef2f3" stroke="#172230" stroke-width="3"/>
<line x1="180" y1="290" x2="1110" y2="290" stroke="#bf4b3f" stroke-width="2"/><text x="570" y="275" font-size="19">CUT 327 ±0.5</text>
<line x1="250" y1="470" x2="1035" y2="470" stroke="#156f88" stroke-width="3"/><text x="535" y="505" font-size="18">S–E EFFECTIVE 317 ±1.0</text>
<text x="180" y="570" font-size="17">SECTION: 6061-T6 SQ TUBE 25.4 × 25.4 × 3.175 WALL</text>
<rect x="1140" y="330" width="120" height="120" fill="none" stroke="#172230" stroke-width="3"/><rect x="1155" y="345" width="90" height="90" fill="white" stroke="#172230" stroke-width="2"/>
<text x="180" y="615" font-size="15">S=(-205,-10,70); E=(-522,-10,70); shoulder/elbow offsets 25; insertions 30</text>
<text x="180" y="650" font-size="15">Worst-case resolver stack: effective ±1.0; seat gap ±1.5; cut length ±2.0</text>
<text x="180" y="685" font-size="15">Deburr 0.2–0.5; break edges; straightness 0.75 per 300; record as-built mass and COG.</text>''')
    interface = drawing(
        "A0-D004-R0 — PITCH SHAFT, BEARING, INDEX AND HUB INTERFACES",
        "Section schematic; coordinate and fit tables control over graphic",
        '''<circle cx="380" cy="430" r="190" fill="#eef2f3" stroke="#172230" stroke-width="4"/><circle cx="380" cy="430" r="26" fill="white" stroke="#172230" stroke-width="3"/>
<g fill="none" stroke="#156f88" stroke-width="3"><circle cx="545" cy="335" r="12"/><circle cx="570" cy="375" r="12"/><circle cx="580" cy="430" r="12"/><circle cx="570" cy="485" r="12"/><circle cx="545" cy="525" r="12"/></g>
<line x1="380" y1="430" x2="760" y2="430" stroke="#bf4b3f" stroke-width="3"/><text x="650" y="415" font-size="16">PITCH AXIS +Y</text>
<text x="850" y="240" font-size="16">SHAFT: Ø12 h6 = 11.989–12.000</text><text x="850" y="275" font-size="16">BEARING: 6001-2RS, 12×28×8</text>
<text x="850" y="310" font-size="16">HOUSING: Ø28 H7 = 28.000–28.021</text><text x="850" y="345" font-size="16">INDEX HOLES: Ø8.2 +0.10/-0.00</text>
<text x="850" y="380" font-size="16">SECTOR: Ø90 × 6; HOLES EACH 15°</text><text x="850" y="415" font-size="16">HUB SOCKET: 25.50 +0.15/-0.00</text>
<text x="850" y="465" font-size="15">2× M6-10.9 PINCH BOLTS</text><text x="850" y="500" font-size="15">SHAFT: CAPTURED END WASHERS + LOCKNUT</text>
<text x="850" y="535" font-size="15">HARD STOPS: ACETAL PADS AT -32°/+92°</text><text x="850" y="570" font-size="15">INDEX ENGAGEMENT DEPTH ≥6 mm</text>
<text x="850" y="625" font-size="14">Procurement lot and actual bearing/plunger dimensions</text><text x="850" y="650" font-size="14">must be checked before machining mating features.</text>''')
    return {
        "A0-D001-fixture-datum.svg": fixture,
        "A0-D002-mechanism-assembly.svg": mechanism,
        "A0-D003-dummy-member.svg": member,
        "A0-D004-cartridge-interface.svg": interface,
    }


def export(output):
    import cadquery as cq

    design = load_design()
    cad_dir, drawing_dir = output / "cad", output / "drawings"
    for folder in (cad_dir / "step", cad_dir / "stl", drawing_dir):
        folder.mkdir(parents=True, exist_ok=True)
    checks = []
    parts = build_parts()
    for name, workplane in parts.items():
        shape = workplane.val()
        if not shape.isValid() or len(shape.Solids()) != 1 or shape.Volume() <= 0:
            raise ValueError(f"invalid solid: {name}")
        step = cad_dir / "step" / f"{name}.step"
        stl = cad_dir / "stl" / f"{name}.stl"
        cq.exporters.export(shape, str(step))
        cq.exporters.export(shape, str(stl), tolerance=0.05, angularTolerance=0.1)
        reloaded = cq.importers.importStep(str(step)).val()
        if not reloaded.isValid() or not math.isclose(reloaded.Volume(), shape.Volume(), rel_tol=1e-6):
            raise ValueError(f"STEP round trip failed: {name}")
        box = shape.BoundingBox()
        checks.append({"part": name, "valid_solid": True, "step_round_trip": "pass",
                       "bounds_mm": [box.xlen, box.ylen, box.zlen], "volume_mm3": shape.Volume()})

    for name, content in drawings(design).items():
        (drawing_dir / name).write_text(content, encoding="utf-8")
    with (output / "BOM.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["item_id", "qty", "description", "material_or_standard", "size_or_procurement_note"])
        writer.writerows(bom_rows())
    manifest = {
        "packet_id": design["packet_id"],
        "revision": "A0-R0",
        "status": "shop_review_candidate_not_fabrication_released",
        "fabrication_released": False,
        "physical_evidence_complete": False,
        "canon_effect": design["canon_effect"],
        "units": "mm",
        "source": str(DESIGN_PATH.relative_to(ROOT)),
        "source_sha256": hashlib.sha256(DESIGN_PATH.read_bytes()).hexdigest(),
        "parts": checks,
        "drawing_files": sorted(drawings(design)),
        "bom_items": len(bom_rows()),
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"Exported {len(parts)} part solids, {len(manifest['drawing_files'])} drawings, and {len(bom_rows())} BOM lines to {output}")
    return manifest


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    export(parser.parse_args().output)
