#!/usr/bin/env python3
"""Export the bounded T56 A0-R2 shoulder article shop-review packet.

CadQuery 2.7.0 is required. Outputs remain unreleased and unqualified.
"""
from __future__ import annotations

import argparse
import csv
import gzip
import hashlib
import io
import json
import math
import re
import tarfile
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from PROJECTS.T56_CARBON.tools.a0_interfaces import stop_geometry, interface_stacks, engineering_report, attachment_screens
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
    f=fabrication(design);c=f['cartridge'];d=f['dummy_member'];q=f['four_bar']
    return [
        ('A0-101',1,'fixture base plate','A36',str(f['fixture']['base_size_xyz'])),
        ('A0-102',2,'four-bar rocker','6061-T6',f"{q['rocker_size_xyz']}; {q['rocker_centers']} centers"),
        ('A0-103',1,'relieved carriage','6061-T6',f"{q['carriage_size_xyz']}; plate center Z={q['carriage_center_z']}"),
        ('A0-104',1,'handed bearing yoke with integral foot','6061-T6 billet',f"one mirrored in Y; {c['yoke_plate_size_xzy']}; foot {c['yoke_foot_size_xyz']}; billet covers full drawn envelope; Ø40 pocket/Ø35 ledge"),
        ('A0-105',1,'offset split hub with integral stop lug','6061-T6 billet',f"body {c['hub_size_xyz']}; integral finished X span {c['stop_lug_radius']+c['stop_lug_radius_tip']-c['hub_center_x_from_S']+c['hub_size_xyz'][0]/2} (add shop stock allowance); open mouth X={c['tube_socket_mouth_x_from_S']}; bottom {c['tube_socket_bottom_x_from_S']}; R{c['stop_lug_radius']} lug"),
        ('A0-106',1,'keyed index sector','6061-T6',f"Ø{c['sector_od']} x {c['sector_thickness']}; Y={c['sector_center_y']}"),
        ('A0-107',2,'pitch stop pad','acetal candidate ONLY',f"{c['stop_pad_size_xyz']}; NO verified energy/travel credit"),
        ('A0-108',1,'dummy member','6061-T6 tube',f"{d['outer']} square x {d['wall']} wall x {d['cut_length']}; insertion 28"),
        ('A0-109',2,'root standoff','6061-T6',str(f['fixture']['standoff_size_xyz'])),
        ('A0-110',2,'scapular bracket provisional envelope','6061-T6',str(c['stop_bracket_size_xyz'])),
        ('A0-111',2,'handed pitch stop carrier frame','6061-T6 billet',f"one mirrored in Y; {c['stop_carrier_beam']} web x {c['stop_carrier_width_y']}; integral feet12 require full D009 envelope billet, not plate16"),
        ('A0-112',2,'stop crossbar with front flat','4140 prehard',f"Ø{c['stop_bar_od']} x {c['stop_bar_span']}; Ø12 journals; 124 shoulder span; M6 ends; flat offset 8"),
        ('A0-113',2,'legacy scapular pad envelope','acetal candidate','20 x15 x10; existing scapular interface remains blocked'),
        ('A0-114',1,'floating-side footed yoke','6061-T6 billet','R2 right yoke; pocket15 ±0.02, inward float1 nominal; unloaded axial ledge1'),
        ('A0-201',1,'pitch shaft with local key seats','4140 prehard',f"Ø17 h6 CANDIDATE x {c['shaft_length']}; 30/6 key seats; M6x1 ends 16 deep"),
        ('A0-202',1,'hub key','hardened key steel','5 x 5 x 30; material/fit acceptance open'),
        ('A0-203',6,'bolted end-capture washer','4140 prehard',f"Ø{c['end_retainer_od_width'][0]} x 3; Ø6.6 clearance"),
        ('A0-204',1,'sector key','hardened key steel',f"5 x 5 x {c['sector_thickness']}"),
        ('A0-205',2,'inner-ring spacer','4140 prehard',f"Ø21.6/Ø17.1 x {c['inner_spacer_length']}"),
        ('A0-206',1,'left outer spacer','4140 prehard',f"Ø21.6/Ø17.1 x {c['outer_spacer_left_length']}"),
        ('A0-207',1,'right inner spacer','4140 prehard',f"Ø21.6/Ø17.1 x {c['outer_spacer_right_inner_length']}"),
        ('A0-208',1,'right outer spacer','4140 prehard',f"Ø21.6/Ø17.1 x {c['outer_spacer_right_outer_length']}"),
        ('A0-301',2,'deep-groove sealed bearing','SKF 6203-2RSH',f"{c['bearing_id_od_width']}; C0=4750 N; C=9950 N; CN/fit approval HOLD"),
        ('A0-302',4,'pivot shoulder bolt envelope','class 10.9','M10 x1.5; actual shoulder/head/washer stack HOLD'),
        ('A0-303',4,'pivot locknut','all-metal prevailing torque','M10 x1.5; supplier-open'),
        ('A0-304',2,'index plunger envelope','supplier-open','8 pin; support/thread/nose HOLD; no impact-credit'),
        ('A0-305',1,'carriage clamp','supplier-open','M8x1.25; grounding/attachment HOLD'),
        ('A0-306',2,'hub pinch screw','class 10.9, ISO 4762 candidate','M6x1 across Y; 20 mm engagement in far cheek; length/locking reviewed before purchase'),
        ('A0-307',1,'secondary tether envelope','steel rope','6 mm; rating >=1 kN; route/end fittings HOLD'),
        ('A0-308',4,'bench screw','class 10.9','M10; reviewed bench/length/locking HOLD'),
        ('A0-309',1,'passive friction envelope','supplier-open','>=7 N m; grounded reaction/coupling/preload HOLD'),
        ('A0-310',1,'locating-side C cap','6061-T6',f"Ø60/Ø35; 4 flange; {c['bearing_cap_nose_length']} nose; bottom Z=-10"),
        ('A0-311',1,'floating-side C cap','6061-T6',f"Ø60/Ø35; 4 flange; {c['bearing_float_cap_nose_length']} nose; bottom Z=-10"),
        ('A0-312',4,'yoke foot screw','class 10.9, ISO 4762 candidate','M6x1 x20; 8 mm carriage thread; dry torque/locking approval HOLD'),
        ('A0-313',6,'cap screw','class 10.9, ISO 4762 candidate','M4x0.7 x10; 6 mm yoke engagement'),
        ('A0-314',6,'shaft/crossbar end screw','class 10.9, ISO 4762 candidate','M6x1 x16; 3 mm washer; 13 mm thread engagement; approved locking required'),
        ('A0-315',4,'stop carrier foot screw','class 10.9, ISO 4762 candidate','M6x1 x20; 8 mm carriage engagement; inspect underside clearance'),
        ('A0-316',4,'recessed pad screw','class 10.9, low-head candidate','M3x0.5 x14; Ø5.7 x1.65 head envelope; delivered head must fit Ø6 x2 recess'),
        ('A0-401',1,'galvanic isolation kit','PET + primer','0.5 PET; installed datum stack review HOLD'),
        ('A0-402',1,'selective end-shim kit','ground steel','annular stack shims Ø21.6/Ø17.1 up to0.75; shaft-face standoffs Ø16.8/Ø6.6 up to0.50; select measured0.05..0.15 end play'),
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
    # Open-sided relief for the actual hub/member sweep. S is 20 mm left
    # of the carriage-pivot line; stations and pivot axes remain unchanged.
    relief_end = q['carriage_relief_x_from_S'] - 20
    carriage = carriage.cut(cq.Workplane('XY').box(cs[0]+2, 2*q['carriage_relief_y_half'], cs[2]+2).translate((relief_end-(cs[0]+2)/2,0,0)))
    lug_relief_end=c['stop_lug_radius']+c['stop_lug_radius_tip']-2-20
    carriage=carriage.cut(cq.Workplane('XY').box(cs[0]+2,c['stop_lug_width']+4,cs[2]+2).translate((lug_relief_end-(cs[0]+2)/2,0,0)))
    for y in (-50,50):
        carriage=carriage.cut(cq.Workplane('XY').box(25,44,cs[2]+2).translate((56.5,y,0)))
    for ypos in c['yoke_plate_center_y']:
        sign = -1 if ypos < design['mechanism']['humeral_stage']['joint_center_mm'][1] else 1
        for x in c['yoke_foot_holes_x']:
            for y in c['yoke_foot_holes_y']:
                carriage = carriage.cut(cq.Solid.makeCylinder(2.5,cs[2]+2,cq.Vector(x-20,ypos+10+sign*y,-cs[2]/2-1),cq.Vector(0,0,1)))
    for x in c['stop_carrier_foot_holes_x']:
        for y in (-c['stop_carrier_foot_y'],c['stop_carrier_foot_y']):
            carriage=carriage.cut(cq.Solid.makeCylinder(2.5,cs[2]+2,cq.Vector(x-20,y,-cs[2]/2-1),cq.Vector(0,0,1)))

    yx, yz, yt = c["yoke_plate_size_xzy"]
    bottom=c['yoke_bottom_z_from_S']
    yoke = cq.Workplane('XY').box(yx,yt,yz).translate((0,0,bottom+yz/2))
    foot=c['yoke_foot_size_xyz']
    yoke=yoke.union(cq.Workplane('XY').box(*foot).translate((0,c['yoke_foot_center_y'],bottom+foot[2]/2)))
    # Right-hand part; left instance is reflected in Y. Ledge is inboard.
    yoke=yoke.cut(cq.Solid.makeCylinder(c['bearing_ledge_bore']/2,yt+2,cq.Vector(0,-yt/2-1,0),cq.Vector(0,1,0)))
    yoke=yoke.cut(cq.Solid.makeCylinder(c['bearing_housing_diameter']/2,c['bearing_pocket_depth']+1,cq.Vector(0,yt/2-c['bearing_pocket_depth'],0),cq.Vector(0,1,0)))
    for x in c['yoke_foot_holes_x']:
        for y in c['yoke_foot_holes_y']:
            yoke=yoke.cut(cq.Solid.makeCylinder(c['yoke_mount_clearance']/2,foot[2]+2,cq.Vector(x,y,bottom-1),cq.Vector(0,0,1)))
    yoke=yoke.cut(cq.Workplane('XY').box(50,40,foot[2]+2).translate((c['yoke_foot_outer_toe_xmax']+25,28,bottom+foot[2]/2)))
    # Access around the existing carriage shoulder-bolt axis.
    yoke=yoke.cut(cq.Solid.makeCylinder(10,foot[2]+2,cq.Vector(20,20,bottom-1),cq.Vector(0,0,1)))
    cap_holes=[(c['bearing_cap_bolt_circle']/2*math.cos(math.radians(a)),c['bearing_cap_bolt_circle']/2*math.sin(math.radians(a))) for a in c['bearing_cap_hole_angles']]
    for x,z in cap_holes:
        yoke=yoke.cut(cq.Solid.makeCylinder(1.65,8,cq.Vector(x,yt/2-7,z),cq.Vector(0,1,0)))

    hx, hy, hz = c["hub_size_xyz"]
    xmin,xmax=c['hub_center_x_from_S']-hx/2,c['hub_center_x_from_S']+hx/2
    bevel=c['hub_corner_bevel']
    hub=cq.Workplane('XZ').polyline([(xmin,-hz/2+bevel),(xmin+bevel,-hz/2),(xmax,-hz/2),(xmax,hz/2),(xmin+bevel,hz/2),(xmin,hz/2-bevel)]).close().extrude(hy/2,both=True)
    shaft_void = cq.Solid.makeCylinder(c["shaft_diameter"] / 2, hy + 2, cq.Vector(0, -hy / 2 - 1, 0), cq.Vector(0, 1, 0))
    hub = hub.cut(shaft_void)
    # Tube extends toward -X: the -40 mouth must be OPEN, the -12 bottom
    # closes at its insertion end. R1 reversed those physical functions.
    socket_length = c['tube_socket_bottom_x_from_S']-c['tube_socket_mouth_x_from_S']+1
    socket_center = (c['tube_socket_bottom_x_from_S']+c['tube_socket_mouth_x_from_S']-1)/2
    socket = cq.Workplane("XY").box(socket_length, c["tube_socket_across_flats"], c["tube_socket_across_flats"]).translate((socket_center, 0, 0))
    hub = hub.cut(socket)
    hub = hub.cut(cq.Workplane("XY").box(socket_length + 4, 1.5, hz / 2).translate((socket_center, 0, hz / 4)))
    hub = hub.cut(cq.Workplane("XY").box(5, hy + 2, 5).translate((0, 0, c["shaft_diameter"] / 2)))
    for xpos in c["hub_pinch_bolt_x_from_S"]:
        hub = hub.cut(cq.Solid.makeCylinder(c['hub_pinch_bolt_diameter']/2,hy/2+1,cq.Vector(xpos,-hy/2-1,c['hub_pinch_bolt_z']),cq.Vector(0,1,0)))
        hub = hub.cut(cq.Solid.makeCylinder(2.5,hy/2+1,cq.Vector(xpos,0,c['hub_pinch_bolt_z']),cq.Vector(0,1,0)))
    # Integral lug routes stop torque directly into the hub, avoiding a
    # fictitious shaft-axis couple assigned to the radial bearing pair.
    r,tip,lw=c['stop_lug_radius'],c['stop_lug_radius_tip'],c['stop_lug_width']
    lug=cq.Workplane('XZ').polyline([(15,-c['stop_lug_root_half_height']),(r-15,-tip+.5),(r,-tip+.5),(r,tip-.5),(r-15,tip-.5),(15,c['stop_lug_root_half_height'])]).close().extrude(lw/2,both=True)
    lug=lug.union(cq.Workplane('XZ').circle(tip).extrude(lw/2,both=True).translate((r,0,0)))
    hub=hub.union(lug)

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
    sector_relative_y=c['sector_center_y']-design['mechanism']['humeral_stage']['joint_center_mm'][1]
    shaft=shaft.cut(cq.Workplane('XY').box(5,c['sector_thickness'],2.5).translate((0,sector_relative_y,c['shaft_diameter']/2-1.25)))
    for sign in (-1,1):
        shaft=shaft.cut(cq.Solid.makeCylinder(c['shaft_end_tap_drill']/2,c['shaft_end_thread_depth'],cq.Vector(0,sign*c['shaft_length']/2,0),cq.Vector(0,-sign,0)))
    kw, kh, kl = c["shaft_key_width_height_length"]
    key = cq.Workplane("XY").box(kw, kl, kh)
    bearing = cq.Workplane("XZ").circle(c["bearing_id_od_width"][1] / 2).circle(c["bearing_id_od_width"][0] / 2).extrude(c["bearing_id_od_width"][2] / 2, both=True)
    friction = cq.Workplane("XZ").circle(c["friction_stack_envelope_od_width"][0] / 2).circle(c["friction_stack_clearance_bore"] / 2).extrude(c["friction_stack_envelope_od_width"][1] / 2, both=True)
    retainer = cq.Workplane("XZ").circle(c["end_retainer_od_width"][0] / 2).circle(c["end_retainer_hole"] / 2).extrude(c["end_retainer_od_width"][1] / 2, both=True)
    # Pad local +Z is its thickness/reaction normal. Holes are outside the
    # 12 mm-wide lug contact strip; recessed heads cannot become stop faces.
    stop = cq.Workplane("XY").box(*c["stop_pad_size_xyz"])
    for y in c['stop_pad_screw_y']:
        stop=stop.cut(cq.Solid.makeCylinder(c['stop_pad_screw_hole']/2,12,cq.Vector(0,y,-6),cq.Vector(0,0,1)))
        stop=stop.cut(cq.Solid.makeCylinder(c['stop_pad_counterbore']/2,c['stop_pad_counterbore_depth'],cq.Vector(0,y,-c['stop_pad_size_xyz'][2]/2),cq.Vector(0,0,1)))
    bracket = cq.Workplane("XY").box(*c["stop_bracket_size_xyz"])
    plunger = cq.Workplane("XZ").circle(c["plunger_envelope_body_pin_length"][0] / 2).extrude(c["plunger_envelope_body_pin_length"][2] / 2, both=True)
    pivot_bolt = cq.Workplane("XY").circle(5).extrude(10, both=True)
    def cap(nose):
        part=cq.Workplane('XZ').circle(c['bearing_cap_od']/2).circle(c['bearing_cap_bore']/2).extrude(c['bearing_cap_thickness']/2,both=True)
        # Shape coordinates have flange centered at Y=0 and nose toward -Y.
        part=part.union(cq.Workplane('XZ').circle(c['bearing_cap_nose_od']/2).circle(c['bearing_cap_bore']/2).extrude(nose/2,both=True).translate((0,-(c['bearing_cap_thickness']+nose)/2,0)))
        for x,z in cap_holes:
            part=part.cut(cq.Solid.makeCylinder(c['bearing_cap_hole']/2,c['bearing_cap_thickness']+2,cq.Vector(x,-c['bearing_cap_thickness']/2-1,z),cq.Vector(0,1,0)))
        part=part.cut(cq.Workplane('XY').box(80,20,40).translate((0,0,c['bearing_cap_bottom_z']-20)))
        return part
    bearing_cap=cap(c['bearing_cap_nose_length'])
    float_cap=cap(c['bearing_float_cap_nose_length'])
    def spacer(length):
        return cq.Workplane('XZ').circle(c['inner_spacer_od']/2).circle(c['inner_spacer_bore']/2).extrude(length/2,both=True)
    def screw(diameter,length,head_d,head_h):
        # Simplified cylindrical thread envelope along +Z, head at Z=0.
        return cq.Workplane('XY').circle(diameter/2).extrude(-length).union(cq.Workplane('XY').circle(head_d/2).extrude(head_h))
    def beam(p,q,width):
        dx,dz=q[0]-p[0],q[1]-p[1]; length=math.hypot(dx,dz)
        return cq.Workplane('XY').box(width,c['stop_carrier_width_y'],length).rotate((0,0,0),(0,1,0),math.degrees(math.atan2(dx,dz))).translate(((p[0]+q[0])/2,0,(p[1]+q[1])/2))
    contacts=stop_geometry(design); lo,hi=[v['backing_center_from_S_mm'] for v in contacts]
    nodes=[(lo[0],lo[2]),(lo[0],c['stop_carrier_top_z']),(c['stop_carrier_x'],c['stop_carrier_top_z']),(c['stop_carrier_x'],c['stop_carrier_bottom_z']),(-20,c['stop_carrier_bottom_z']),(hi[0],hi[2])]
    carrier=None
    for a,b in zip(nodes,nodes[1:]):
        segment=beam(a,b,c['stop_carrier_beam'])
        carrier=segment if carrier is None else carrier.union(segment)
    for row in contacts:
        x,_,z=row['backing_center_from_S_mm']
        boss=cq.Workplane('XZ').circle(16).extrude(c['stop_carrier_width_y']/2,both=True).translate((x,0,z))
        carrier=carrier.union(boss)
        carrier=carrier.cut(cq.Solid.makeCylinder(6.05,c['stop_carrier_width_y']+2,cq.Vector(x,-c['stop_carrier_width_y']/2-1,z),cq.Vector(0,1,0)))
    foot_z=q['carriage_center_z']+cs[2]/2-design['mechanism']['humeral_stage']['joint_center_mm'][2]
    x0,x1=c['stop_carrier_foot_x']
    ear=cq.Workplane('XY').box(x1-x0,c['stop_carrier_foot_size_y'],12).translate(((x0+x1)/2,(50-c['stop_carrier_frame_y']+8)/2,foot_z+6))
    for x in c['stop_carrier_foot_holes_x']:
        ear=ear.cut(cq.Solid.makeCylinder(3.3,14,cq.Vector(x,c['stop_carrier_foot_y']-c['stop_carrier_frame_y'],foot_z-1),cq.Vector(0,0,1)))
    carrier=carrier.union(ear)
    # Right-sector passage through the inboard foot bridge; same relieved
    # geometry is mirrored for the left frame. The frame web stays outside R46.
    local_sector_y=c['sector_center_y']-(design['mechanism']['humeral_stage']['joint_center_mm'][1]+c['stop_carrier_frame_y'])
    carrier=carrier.cut(cq.Solid.makeCylinder(c['sector_od']/2+1,8,cq.Vector(0,local_sector_y-4,0),cq.Vector(0,1,0)))
    carrier=carrier.cut(cq.Workplane('XY').box(300,100,100).translate((0,0,1-design['mechanism']['humeral_stage']['joint_center_mm'][2]-50)))
    stop_bar=cq.Workplane('XZ').circle(c['stop_bar_od']/2).extrude(c['stop_bar_shoulder_span']/2,both=True)
    stop_bar=stop_bar.union(cq.Workplane('XZ').circle(c['stop_bar_journal']/2).extrude(c['stop_bar_span']/2,both=True))
    stop_bar=stop_bar.cut(cq.Workplane('XY').box(30,c['stop_bar_shoulder_span'],10).translate((0,0,-c['stop_bar_flat_offset']-5)))
    for y in c['stop_pad_screw_y']:
        stop_bar=stop_bar.cut(cq.Solid.makeCylinder(1.25,7,cq.Vector(0,y,-c['stop_bar_flat_offset']-1),cq.Vector(0,0,1)))
    for sign in (-1,1):
        stop_bar=stop_bar.cut(cq.Solid.makeCylinder(2.5,16,cq.Vector(0,sign*c['stop_bar_span']/2,0),cq.Vector(0,-sign,0)))
    tether = cq.Workplane("YZ").circle(3).extrude(60)
    return {
        "A0-101_fixture_base": base, "A0-102_rocker": rocker, "A0-103_carriage": carriage,
        "A0-104_yoke_side": yoke, "A0-105_output_hub": hub, "A0-106_index_sector": sector,
        "A0-107_stop_pad": stop, "A0-108_dummy_member": dummy, "A0-109_root_standoff": standoff,
        "A0-110_stop_bracket": bracket, "A0-201_pitch_shaft": shaft, "A0-202_key": key,
        "A0-203_end_retainer": retainer, "A0-301_bearing_envelope": bearing,
        "A0-304_plunger_envelope": plunger, "A0-309_friction_stack_envelope": friction,
        "A0-302_pivot_bolt_envelope": pivot_bolt, "A0-307_tether_envelope": tether,
        "A0-310_bearing_cap": bearing_cap,
        'A0-311_floating_cap':float_cap,'A0-111_pitch_stop_carrier':carrier,
        'A0-112_stop_crossbar':stop_bar,
        'A0-204_sector_key':cq.Workplane('XY').box(kw,c['sector_thickness'],kh),
        'A0-205_inner_spacer':spacer(c['inner_spacer_length']),
        'A0-206_left_outer_spacer':spacer(c['outer_spacer_left_length']),
        'A0-207_right_inner_spacer':spacer(c['outer_spacer_right_inner_length']),
        'A0-208_right_outer_spacer':spacer(c['outer_spacer_right_outer_length']),
        'A0-312_mount_screw':screw(5,c['yoke_mount_screw_length'],10,6),
        'A0-313_cap_screw':screw(3.3,c['bearing_cap_screw_length'],7,4),
        'A0-314_end_screw':screw(5,16,10,6),
        'A0-315_carrier_screw':screw(5,20,10,6),
        'A0-316_pad_screw':screw(2.5,14,5.7,1.65),
        'A0-114_floating_yoke':yoke.cut(cq.Solid.makeCylinder(c['bearing_housing_diameter']/2,c['bearing_floating_pocket_depth']+1,cq.Vector(0,yt/2-c['bearing_floating_pocket_depth'],0),cq.Vector(0,1,0))),
        'A0-113_scapular_pad_envelope':cq.Workplane('XY').box(20,15,10),
    }


def located_instances(parts, design=None, include_provisional=False):
    """Neutral located instances; tapped/threaded hardware is explicitly simplified."""
    design=design or load_design();f=fabrication(design);c=f['cartridge'];q=f['four_bar']
    stage=design['mechanism']['scapular_stage'];s=design['mechanism']['humeral_stage']['joint_center_mm']
    p,a=stage['posterior_root_mm'],stage['anterior_root_mm'];b,cc=stage['posterior_carriage_pivot_mm'],stage['anterior_carriage_pivot_mm']
    mid=lambda u,v:tuple((u[i]+v[i])/2 for i in range(3))
    rows=[]
    def add(iid,pid,point=(0,0,0),shape=None):
        rows.append((iid,pid,(shape if shape is not None else parts[pid]).translate(point)))
    add('fixture','A0-101_fixture_base',(-130,0,0))
    for name,point in (('P',p),('A',a)):
        add('root_standoff_'+name,'A0-109_root_standoff',(point[0],point[1],0))
    add('rocker_PB','A0-102_rocker',mid(p,b));add('rocker_AC','A0-102_rocker',mid(a,cc))
    add('carriage','A0-103_carriage',(b[0],(b[1]+cc[1])/2,q['carriage_center_z']))
    for name,point in (('P',p),('A',a),('B',b),('C',cc)):
        add('pivot_bolt_'+name,'A0-302_pivot_bolt_envelope',point)
    for i,y in enumerate(c['yoke_plate_center_y'],1):
        sign=-1 if y<s[1] else 1
        yoke_pid='A0-104_yoke_side' if sign<0 else 'A0-114_floating_yoke'
        yoke=parts[yoke_pid]
        if sign<0:yoke=yoke.mirror('XZ')
        add('yoke_'+str(i),yoke_pid,(s[0],y,s[2]),yoke)
        add('bearing_'+str(i),'A0-301_bearing_envelope',(s[0],y,s[2]))
        pid='A0-310_bearing_cap' if sign<0 else 'A0-311_floating_cap'
        cap=parts[pid]
        if sign<0:cap=cap.mirror('XZ')
        cap_y=y+sign*(c['yoke_plate_size_xzy'][2]/2+c['bearing_cap_thickness']/2)
        add('bearing_cap_'+str(i),pid,(s[0],cap_y,s[2]),cap)
        for n,angle in enumerate(c['bearing_cap_hole_angles'],1):
            t=math.radians(angle);r=c['bearing_cap_bolt_circle']/2
            screw=parts['A0-313_cap_screw'].rotate((0,0,0),(1,0,0),-sign*90)
            add(f'cap_screw_{i}_{n}','A0-313_cap_screw',(s[0]+r*math.cos(t),y+sign*(c['yoke_plate_size_xzy'][2]/2+c['bearing_cap_thickness']),s[2]+r*math.sin(t)),screw)
        for n,(x,yy) in enumerate(((x,yy) for x in c['yoke_foot_holes_x'] for yy in c['yoke_foot_holes_y']),1):
            add(f'mount_screw_{i}_{n}','A0-312_mount_screw',(s[0]+x,y+sign*yy,s[2]+c['yoke_bottom_z_from_S']+c['yoke_foot_size_xyz'][2]))
    for name,pid in (('hub','A0-105_output_hub'),('shaft','A0-201_pitch_shaft')):add(name,pid,s)
    add('key','A0-202_key',(s[0],s[1],s[2]+c['shaft_diameter']/2))
    add('sector','A0-106_index_sector',(s[0],c['sector_center_y'],s[2]))
    add('sector_key','A0-204_sector_key',(s[0],c['sector_center_y'],s[2]+c['shaft_diameter']/2))
    add('friction_stack','A0-309_friction_stack_envelope',(s[0],c['friction_stack_center_y'],s[2]))
    for i,y in enumerate(c['yoke_plate_center_y'],1):
        sign=-1 if y<s[1] else 1
        inner=y-sign*c['bearing_id_od_width'][2]/2-sign*c['inner_spacer_length']/2
        add('inner_spacer_'+str(i),'A0-205_inner_spacer',(s[0],inner,s[2]))
    bw=c['bearing_id_od_width'][2];half=c['shaft_length']/2
    for iid,pid,y in (('left_outer_spacer','A0-206_left_outer_spacer',s[1]-half+c['outer_spacer_left_length']/2),('right_inner_spacer','A0-207_right_inner_spacer',c['yoke_plate_center_y'][1]+bw/2+c['outer_spacer_right_inner_length']/2),('right_outer_spacer','A0-208_right_outer_spacer',c['sector_center_y']+c['sector_thickness']/2+c['outer_spacer_right_outer_length']/2)):
        add(iid,pid,(s[0],y,s[2]))
    for i,sign in enumerate((-1,1),1):
        y=s[1]+sign*(c['shaft_length']/2+c['end_retainer_od_width'][1]/2)
        add('retainer_'+str(i),'A0-203_end_retainer',(s[0],y,s[2]))
        screw=parts['A0-314_end_screw'].rotate((0,0,0),(1,0,0),-sign*90)
        add('end_screw_'+str(i),'A0-314_end_screw',(s[0],s[1]+sign*(c['shaft_length']/2+c['end_retainer_od_width'][1]),s[2]),screw)
    member=parts['A0-108_dummy_member'].rotate((0,0,0),(0,0,1),180)
    add('dummy_member','A0-108_dummy_member',(s[0]+f['dummy_member']['cut_start_x_from_S'],s[1],s[2]),member)
    for sign in (-1,1):
        frame=parts['A0-111_pitch_stop_carrier']
        if sign<0:frame=frame.mirror('XZ')
        add('pitch_stop_carrier_'+str(sign),'A0-111_pitch_stop_carrier',(s[0],s[1]+sign*c['stop_carrier_frame_y'],s[2]),frame)
    for i,row in enumerate(stop_geometry(design),1):
        point=[s[j]+row['pad_center_from_S_mm'][j] for j in range(3)]
        pad=parts['A0-107_stop_pad'].rotate((0,0,0),(0,1,0),row['pad_rotation_y_deg'])
        add('pitch_stop_pad_'+str(i),'A0-107_stop_pad',point,pad)
        bar=parts['A0-112_stop_crossbar'].rotate((0,0,0),(0,1,0),row['pad_rotation_y_deg'])
        add('stop_crossbar_'+str(i),'A0-112_stop_crossbar',tuple(s[j]+row['backing_center_from_S_mm'][j] for j in range(3)),bar)
        for n,y in enumerate(c['stop_pad_screw_y'],1):
            screw=parts['A0-316_pad_screw'].rotate((0,0,0),(1,0,0),180).translate((0,y,-c['stop_pad_size_xyz'][2]/2+c['stop_pad_counterbore_depth'])).rotate((0,0,0),(0,1,0),row['pad_rotation_y_deg'])
            add(f'pad_screw_{i}_{n}','A0-316_pad_screw',point,screw)
        for sign in (-1,1):
            bar_point=[s[j]+row['backing_center_from_S_mm'][j] for j in range(3)]
            bar_point[1]+=sign*(c['stop_bar_span']/2+c['end_retainer_od_width'][1]/2)
            add(f'bar_washer_{i}_{sign}','A0-203_end_retainer',bar_point)
            bar_point[1]+=sign*c['end_retainer_od_width'][1]/2
            screw=parts['A0-314_end_screw'].rotate((0,0,0),(1,0,0),-sign*90)
            add(f'bar_end_screw_{i}_{sign}','A0-314_end_screw',bar_point,screw)
    for i,(x,y) in enumerate(((x,y) for x in c['stop_carrier_foot_holes_x'] for y in (-c['stop_carrier_foot_y'],c['stop_carrier_foot_y'])),1):
        add('carrier_screw_'+str(i),'A0-315_carrier_screw',(s[0]+x,s[1]+y,q['carriage_center_z']+q['carriage_size_xyz'][2]/2+12))
    # Remove physically undefined placements from the issued subassembly.
    # Their original envelopes are still audited in the discrepancy report.
    if not include_provisional:return rows
    # These envelopes remain intentionally provisional and separately blocked.
    add('index_plunger','A0-304_plunger_envelope',(s[0]+35,c['sector_center_y']+30,s[2]))
    for i,y in enumerate((-72,52),1):
        add('scapular_stop_pad_'+str(i),'A0-113_scapular_pad_envelope',(-185,y,35))
        add('scapular_stop_bracket_'+str(i),'A0-110_stop_bracket',(-168,y,35))
    add('scapular_index_plunger','A0-304_plunger_envelope',(-185,40,35))
    add('secondary_tether','A0-307_tether_envelope',c['tether_endpoints_neutral'][0],parts['A0-307_tether_envelope'].rotate((0,0,0),(0,0,1),180))
    return rows


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


def assembly_interface_report(parts, design):
    """Exact finite-pose tests, explicit scope; no contact-class exemptions."""
    rows=located_instances(parts,design,include_provisional=True);shapes={n:sh.val() for n,_,sh in rows}
    revised=[n for n,_,_ in located_instances(parts,design)]
    legacy=[n for n in shapes if n not in revised]
    mating_gaps={n:round(shapes[n].distance(shapes['carriage']),9)
                 for n in ('yoke_1','yoke_2','pitch_stop_carrier_-1','pitch_stop_carrier_1')}
    def overlap(a,b):
        x,y=a.BoundingBox(),b.BoundingBox()
        if x.xmax<y.xmin or y.xmax<x.xmin or x.ymax<y.ymin or y.ymax<x.ymin or x.zmax<y.zmin or y.zmax<x.zmin:return 0.
        return max(0.,a.intersect(b).Volume())
    bad=[];unreviewed=[];pairs=0
    ids=list(shapes)
    for j,a in enumerate(ids):
        for b in ids[j+1:]:
            is_reviewed=a in revised and b in revised
            pairs+=is_reviewed
            volume=overlap(shapes[a],shapes[b])
            if volume>1e-5:
                (bad if is_reviewed else unreviewed).append({'a':a,'b':b,'penetration_mm3':round(volume,6)})
    s=design['mechanism']['humeral_stage']['joint_center_mm'];axis=(s[0],s[1]+1,s[2])
    targets=['carriage','yoke_1','yoke_2','bearing_cap_1','bearing_cap_2','pitch_stop_carrier_-1','pitch_stop_carrier_1','pitch_stop_pad_1','pitch_stop_pad_2','stop_crossbar_1','stop_crossbar_2']
    sweep=[]
    for angle in design['fabrication_geometry']['assembly']['sampled_pitch_angles_deg']:
        penetrations=[];gaps={}
        for moving in ('hub','dummy_member'):
            pose=shapes[moving].rotate(s,axis,angle)
            for target in targets:
                v=overlap(pose,shapes[target])
                if v>1e-5:penetrations.append({'a':moving,'b':target,'penetration_mm3':round(v,6)})
                if moving=='hub':gaps[target]=round(pose.distance(shapes[target]),6)
        sweep.append({'pitch_deg':angle,'unintended_penetrations':penetrations,'hub_gaps_mm':gaps})
    fixture_hits=[]
    root_hits=[]
    stage=design['mechanism']['scapular_stage'];r=stage['rocker_center_distance_mm']
    for scap in design['fabrication_geometry']['assembly']['sampled_scapular_angles_deg']:
        t=math.radians(scap);delta=(r*(1-math.cos(t)),r*math.sin(t),0)
        root_targets={n:shapes[n] for n in ('root_standoff_P','root_standoff_A','pivot_bolt_P','pivot_bolt_A')}
        for name,pivot in (('rocker_PB',stage['posterior_root_mm']),('rocker_AC',stage['anterior_root_mm'])):
            root_targets[name]=shapes[name].rotate(pivot,(pivot[0],pivot[1],pivot[2]+1),-scap)
        for name in ['carriage','yoke_1','yoke_2','pitch_stop_carrier_-1','pitch_stop_carrier_1']+[n for n in shapes if n.startswith(('mount_screw_','carrier_screw_'))]:
            pose=shapes[name].translate(delta)
            for target,solid in root_targets.items():
                v=overlap(pose,solid)
                if v>1e-5:root_hits.append({'scapular_deg':scap,'a':name,'b':target,'penetration_mm3':round(v,6)})
        for angle in design['fabrication_geometry']['assembly']['sampled_pitch_angles_deg']:
            for name in ('hub','dummy_member'):
                pose=shapes[name].rotate(s,axis,angle).translate(delta)
                v=overlap(pose,shapes['fixture'])
                if v>1e-5:fixture_hits.append({'scapular_deg':scap,'pitch_deg':angle,'part':name,'penetration_mm3':round(v,6)})
    return {'method':'CadQuery solid intersections; threshold 1e-5 mm3; Euclidean minimum nominal distances',
            'required_carriage_mating_gaps_mm':mating_gaps,
            'revised_neutral_pair_count':pairs,'revised_neutral_unintended_penetrations':bad,
            'pitch_pose_count':len(sweep),'pitch_pose_checks':sweep,'fixture_pose_count':35,'fixture_penetrations':fixture_hits,
            'scapular_root_pose_count':5,'scapular_root_penetrations':root_hits,
            'stop_contacts':stop_geometry(design),'unreviewed_legacy_instances':legacy,'legacy_or_mixed_penetrations':unreviewed,
            'result':'pass_revised_interfaces_at_samples_only' if not bad and not fixture_hits and not root_hits and not any(v>1e-6 for v in mating_gaps.values()) and not any(r['unintended_penetrations'] for r in sweep) else 'blocked_revised_collision_or_unseated_mount',
            'whole_article_clearance_approved':False,'continuous_clearance_proof':False,
            'limitations':['legacy/mixed penetrations are release blockers, not intentional-contact exemptions',
                          'relative cartridge check uses parallel-four-bar translation invariance; seven pitch poses only',
                          'full rocker/bolt motion, supplier internals, guards and tools remain unverified',
                          'nominal CAD gaps are not tolerance/thermal or structural qualification']}


def normalize_step(path):
    content = path.read_text(encoding="utf-8")
    content = re.sub(r"(FILE_NAME\('Open CASCADE Shape Model',')[^']+(')", r"\g<1>1970-01-01T00:00:00\2", content)
    path.write_text("\n".join(line.rstrip() for line in content.splitlines()) + "\n", encoding="utf-8")


def deterministic_gzip(source, target):
    with target.open("wb") as handle, gzip.GzipFile(filename="", mode="wb", fileobj=handle, mtime=0) as archive:
        archive.write(source.read_bytes())


def deterministic_step_bundle(step_paths, target):
    with target.open("wb") as handle, gzip.GzipFile(filename="", mode="wb", fileobj=handle, mtime=0) as compressed:
        with tarfile.open(fileobj=compressed, mode="w") as archive:
            for path in sorted(step_paths, key=lambda item: item.name):
                data = path.read_bytes()
                info = tarfile.TarInfo(path.name)
                info.size = len(data); info.mtime = 0; info.mode = 0o644
                info.uid = info.gid = 0; info.uname = info.gname = ""
                archive.addfile(info, io.BytesIO(data))


def drawings(design, parts=None, instances=None):
    from PROJECTS.T56_CARBON.tools.a0_drawings import drawings as render
    parts=parts or build_parts(design)
    return render(design,parts,instances or located_instances(parts,design))


def export(output):
    import cadquery as cq

    design = load_design(); f = fabrication(design)
    cad_dir, drawing_dir, assembly_dir = output / "cad", output / "drawings", output / "assembly"
    for stale_dir in (cad_dir / "step", cad_dir / "stl"):
        if stale_dir.exists():
            for stale in stale_dir.iterdir():
                if stale.is_file():
                    stale.unlink()
            stale_dir.rmdir()
    for folder in (cad_dir / "step", drawing_dir, assembly_dir):
        folder.mkdir(parents=True, exist_ok=True)
    parts = build_parts(design); checks = []; step_paths = []
    for name, workplane in parts.items():
        shape = workplane.val()
        if not shape.isValid() or len(shape.Solids()) != 1 or shape.Volume() <= 0:
            raise ValueError(f"invalid solid: {name}")
        step = cad_dir / "step" / f"{name}.step"
        cq.exporters.export(shape, str(step))
        normalize_step(step)
        reloaded = cq.importers.importStep(str(step)).val()
        if not reloaded.isValid() or not math.isclose(reloaded.Volume(), shape.Volume(), rel_tol=1e-6):
            raise ValueError(f"STEP round trip failed: {name}")
        box = shape.BoundingBox()
        checks.append({"part": name, "valid_solid": True, "step_round_trip": "pass", "bounds_mm": [box.xlen, box.ylen, box.zlen], "volume_mm3": shape.Volume()})
        step_paths.append(step)

    parts_bundle = cad_dir / "A0-R2-parts-step.tar.gz"
    deterministic_step_bundle(step_paths, parts_bundle)
    for step in step_paths:
        step.unlink()
    (cad_dir / "step").rmdir()

    instances = located_instances(parts, design)
    compound = cq.Compound.makeCompound([item[2].val() for item in instances])
    assembly_step = assembly_dir / "A0-R2-neutral-assembly.step"
    cq.exporters.export(compound, str(assembly_step)); normalize_step(assembly_step)
    if not cq.importers.importStep(str(assembly_step)).val().isValid():
        raise ValueError("assembly STEP round trip failed")
    assembly_bundle = assembly_dir / "A0-R2-neutral-assembly.step.gz"
    deterministic_gzip(assembly_step, assembly_bundle)
    assembly_step.unlink()
    instance_rows = []
    for iid, pid, shape in instances:
        box = shape.val().BoundingBox()
        instance_rows.append({"instance_id": iid, "part_id": pid, "valid": shape.val().isValid(),
                              "bounds_mm": {"min": [box.xmin, box.ymin, box.zmin], "max": [box.xmax, box.ymax, box.zmax]}})
    (assembly_dir / "assembly_instances.json").write_text(json.dumps({"frame_id": design["frame_id"], "pose_id": design["pose_id"], "instances": instance_rows}, indent=2) + "\n", encoding="utf-8")
    motion = motion_report(design)
    (assembly_dir / "motion_clearance_report.json").write_text(json.dumps(motion, indent=2) + "\n", encoding="utf-8")

    interfaces=assembly_interface_report(parts,design)
    if interfaces['result']!='pass_revised_interfaces_at_samples_only':
        raise ValueError('A0 revised interfaces contain penetration or an unseated mounting face; see assembly_interface_report')
    (assembly_dir/'interface_check_report.json').write_text(json.dumps(interfaces,indent=2)+'\n')
    (output/'analysis').mkdir(exist_ok=True)
    (output/'analysis/A0_R2_INTERFACES.json').write_text(json.dumps(interface_stacks(design),indent=2)+'\n')
    from PROJECTS.T56_CARBON.tools.a0_load_cases import calculate
    attachment=attachment_screens(design,calculate(design=design)['cases']['LC-SHO-004']['results']['linear_spring_peak_stop_force_N'])
    (output/'analysis/A0_R2_ATTACHMENTS.json').write_text(json.dumps(attachment,indent=2)+'\n')
    engineering,sensitivity=engineering_report(design,load_json(PROJECT/'requirements/a0_article_scope.json'))
    (output/'analysis/A0_R2_ENGINEERING.md').write_text(engineering)
    (output/'analysis/A0_R2_SENSITIVITY.json').write_text(json.dumps({'status':'hypothetical_not_measured','rows':sensitivity},indent=2)+'\n')
    drawing_set = drawings(design,parts,instances)
    for name, content in drawing_set.items():
        (drawing_dir / name).write_text(content, encoding="utf-8")
    with (output / "BOM.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle, lineterminator="\n")
        writer.writerow(["item_id", "qty", "description", "material_or_standard", "size_or_procurement_note"]); writer.writerows(bom_rows(design))
    manifest = {
        "packet_id": design["packet_id"], "revision": f["revision"], "status": "independent_shop_review_candidate_not_fabrication_released",
        "fabrication_released": False, "physical_evidence_complete": False, "canon_effect": design["canon_effect"], "units": "mm",
        "source": str(DESIGN_PATH.relative_to(ROOT)), "source_sha256": hashlib.sha256(DESIGN_PATH.read_bytes()).hexdigest(),
        "source_hashes":{path:hashlib.sha256((ROOT/path).read_bytes()).hexdigest() for path in [
            str(DESIGN_PATH.relative_to(ROOT)), 'PROJECTS/T56_CARBON/requirements/a0_article_scope.json',
            'PROJECTS/T56_CARBON/requirements/a0_analysis_basis.json','PROJECTS/T56_CARBON/requirements/a0_supplier_basis.json',
            'PROJECTS/T56_CARBON/tools/a0_interfaces.py','PROJECTS/T56_CARBON/tools/a0_drawings.py',
            'PROJECTS/T56_CARBON/tools/a0_load_cases.py','bin/export_t56_a0_article.py']},
        "parts": checks, "parts_step_bundle": str(parts_bundle.relative_to(output)),
        "assembly_step_gzip": str(assembly_bundle.relative_to(output)), "assembly_instance_count": len(instances),
        "motion_clearance": {k: motion[k] for k in ("method", "continuous_clearance_proof", "sample_count", "sampled_collision_count", "result")},
        "omitted_provisional_instances":[n for n in interfaces["unreviewed_legacy_instances"] if n not in {v[0] for v in instances}],
        "assembly_interface_result":interfaces["result"], "whole_article_clearance_approved":False,
        "drawing_files": sorted(drawing_set), "bom_items": len(bom_rows(design)), "supplier_dependent_open_items": f["supplier_dependent_open_items"],
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"Exported A0-R2: {len(parts)} parts, {len(instances)} instances, {len(drawing_set)} drawings, {len(bom_rows(design))} BOM lines")
    return manifest


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    export(parser.parse_args().output)
