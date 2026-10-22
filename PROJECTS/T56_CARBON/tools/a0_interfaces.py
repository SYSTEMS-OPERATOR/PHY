"""Small, unit-explicit A0 cartridge interface calculations (no CAD dependency)."""
from __future__ import annotations
import math


def positive(value, name):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
        raise ValueError(f'{name} must be finite and positive')
    return value


def bearing_reactions(force_N, moment_Nmm, centers_y_mm):
    """Rigid two-bearing FBD about S, +Y shaft. M includes r cross F.

    A locates axially, B floats. Bearings cannot react shaft-axis torque.
    Return the unbalanced My explicitly rather than assigning it to radial loads.
    """
    if len(force_N) != 3 or len(moment_Nmm) != 3 or len(centers_y_mm) != 2:
        raise ValueError('three-vector loads and two bearing centers required')
    if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in (*force_N, *moment_Nmm, *centers_y_mm)):
        raise ValueError('finite loads and coordinates required')
    a, b = centers_y_mm; span = positive(b-a, 'bearing span [mm]')
    fx, fy, fz = force_N; mx, my, mz = moment_Nmm
    bx, bz = (mz+a*fx)/span, (-mx+a*fz)/span
    ra, rb = [-fx-bx, -fy, -fz-bz], [bx, 0., bz]
    residual_force = [ra[i]+rb[i]+force_N[i] for i in range(3)]
    residual_moment = [a*ra[2]+b*rb[2]+mx, my, -a*ra[0]-b*rb[0]+mz]
    return {'A_N':ra,'B_N':rb,'radial_N':[math.hypot(ra[0],ra[2]),math.hypot(rb[0],rb[2])],
            'axial_N':[abs(fy),0.], 'force_residual_N':residual_force,
            'moment_residual_Nmm':residual_moment,'torque_requiring_separate_reaction_Nmm':my}


def equivalent_static_load(radial_N, axial_N):
    if min(radial_N, axial_N) < 0 or not all(math.isfinite(v) for v in (radial_N, axial_N)):
        raise ValueError('nonnegative finite load magnitudes required')
    return max(radial_N, 0.6*radial_N+0.5*axial_N)


def stop_geometry(design):
    c = design['fabrication_geometry']['cartridge']
    radius = positive(c['stop_lug_radius'], 'stop radius [mm]')
    tip = positive(c['stop_lug_radius_tip'], 'lug radius [mm]')
    thickness = c['stop_pad_size_xyz'][2]
    out = []
    for i, angle in enumerate(design['mechanism']['humeral_stage']['hard_stop_range_deg']):
        t = math.radians(angle); sign = -1 if i == 0 else 1
        center = [radius*math.cos(t), 0., -radius*math.sin(t)]
        normal = [-sign*math.sin(t), 0., -sign*math.cos(t)]
        # normal points OUT of the admissible arc; reaction on lug is -normal.
        contact = [center[j]+tip*normal[j] for j in range(3)]
        pad_center = [center[j]+(tip+thickness/2)*normal[j] for j in range(3)]
        backing = [center[j]+(tip+thickness+c['stop_bar_flat_offset'])*normal[j] for j in range(3)]
        # Flat contact's perpendicular distance from the shaft axis.
        arm = abs(contact[2]*normal[0]-contact[0]*normal[2])
        out.append({'angle_deg':angle,'lug_center_from_S_mm':center,'outward_normal':normal,
                    'contact_from_S_mm':contact,'pad_center_from_S_mm':pad_center,
                    'backing_center_from_S_mm':backing,'effective_arm_mm':arm,
                    'pad_rotation_y_deg':math.degrees(math.atan2(normal[0],normal[2]))})
    return out


def impact_free_body(design, mass_kg, payload_kg, length_mm, energy_J, travel_mm, curve_area_fraction=0.5):
    """Conditional rigid-body deceleration at a tangent stop, ignoring gravity.

    Distributed equivalent mass at L/2 and payload at L, as in frozen A0 basis.
    alpha = F_stop*r/J. Inertia resultant adds to stop reaction because the
    member lies opposite the integral positive-X lug. Not a measured impact.
    """
    for v,n in ((mass_kg,'mass [kg]'),(length_mm,'length [mm]'),(energy_J,'energy [J]'),(travel_mm,'travel [mm]')):positive(v,n)
    if payload_kg < 0 or not math.isfinite(payload_kg):raise ValueError('payload must be finite and nonnegative')
    if not 0 < curve_area_fraction <= 1:raise ValueError('curve area fraction must be in (0,1]')
    length_m=length_mm/1000; radius_m=stop_geometry(design)[0]['effective_arm_mm']/1000
    inertia=mass_kg*(length_m/2)**2+payload_kg*length_m**2
    first=mass_kg*length_m/2+payload_kg*length_m
    force=energy_J/(curve_area_fraction*travel_mm/1000)
    alpha=force*radius_m/inertia
    inertia_force=first*alpha
    total=force+inertia_force
    s=design['mechanism']['humeral_stage']['joint_center_mm']
    cy=[v-s[1] for v in design['fabrication_geometry']['cartridge']['yoke_plate_center_y']]
    reactions=bearing_reactions([0,0,-total],[0,0,0],cy)
    return {'stop_force_N':force,'stop_torque_Nm':force*radius_m,'inertia_kg_m2':inertia,
            'angular_deceleration_rad_s2':alpha,'inertia_resultant_N':inertia_force,
            'total_radial_load_N':total,'bearing_reactions':reactions,
            'maximum_bearing_radial_N':max(reactions['radial_N']),
            'shaft_midspan_bending_Nmm':total*(cy[1]-cy[0])/4,
            'curve_area_fraction':curve_area_fraction,'usable_travel_mm':travel_mm,
            'status':'conditional_on_unverified_compliance_and_rigid_body_model'}


def interface_stacks(design):
    c=design['fabrication_geometry']['cartridge']; lim=design['fabrication_geometry']['interface_limits']
    low=lambda k:lim[k][0]; high=lambda k:lim[k][1]
    overlap=[(low('bearing_od_incoming')-high('cap_bore'))/2,(high('bearing_od_incoming')-low('cap_bore'))/2]
    gap=lambda nose:[low('pocket_depth')-high(nose)-high('bearing_width_incoming'),high('pocket_depth')-low(nose)-low('bearing_width_incoming')]
    # Compare inner-ring center spacing with independently machined housing spacing.
    inner_span=[low('hub_width')+2*low('inner_spacer_length')+low('bearing_width_incoming'),high('hub_width')+2*high('inner_spacer_length')+high('bearing_width_incoming')]
    mismatch=[inner_span[0]-high('yoke_center_span'),inner_span[1]-low('yoke_center_span')]
    # A locating ring can move through its pocket clearance. B therefore
    # needs independent inward AND outward reserve, not just a wide cavity.
    locating_gap=gap('cap_nose')
    inward=low('floating_pocket_depth')-c['yoke_plate_size_xzy'][2]/2-c['bearing_id_od_width'][2]/2
    outward=c['yoke_plate_size_xzy'][2]/2-high('float_cap_nose')-high('bearing_width_incoming')/2
    inward_demand=max(0,-mismatch[0])+locating_gap[1]
    outward_demand=max(0,mismatch[1])+locating_gap[1]
    # Worst-case complete inner stack; no shim or preload has been selected.
    stack_keys=('hub_width','outer_spacer_left_length','outer_spacer_right_inner_length','outer_spacer_right_outer_length','sector_thickness')
    stack=[sum(lim[k][i] for k in stack_keys)+2*lim['inner_spacer_length'][i]+2*lim['bearing_width_incoming'][i] for i in (0,1)]
    right_float=[low('floating_pocket_depth')-high('float_cap_nose')-high('bearing_width_incoming'),high('floating_pocket_depth')-low('float_cap_nose')-low('bearing_width_incoming')]
    return {'method':'worst_case_interval_arithmetic; independent dimensions; no RSS credit',
            'cap_radial_overlap_mm':overlap,'left_outer_ring_axial_clearance_mm':gap('cap_nose'),
            'right_outer_ring_axial_clearance_mm':right_float,'inner_vs_outer_center_span_error_mm':mismatch,
            'floating_seat_inward_available_mm':inward,'floating_seat_outward_available_mm':outward,
            'floating_seat_inward_demand_mm':inward_demand,'floating_seat_outward_demand_mm':outward_demand,
            'floating_seat_reserve_mm':min(inward-inward_demand,outward-outward_demand),
            'unshimmed_inner_stack_length_mm':stack,
            'unshimmed_stack_minus_shaft_mm':[stack[0]-high('shaft_length'),stack[1]-low('shaft_length')],
            'selected_end_shims_mm':None,
            'shaft_diametral_clearance_mm':[low('bearing_bore_incoming')-high('shaft'),high('bearing_bore_incoming')-low('shaft')],
            'housing_diametral_clearance_mm':[low('housing_bore')-high('bearing_od_incoming'),high('housing_bore')-low('bearing_od_incoming')],
            'spacer_abutment_od_mm':lim['spacer_od'],
            'limitations':['incoming acceptance windows require actual measurements; not catalog tolerance claims',
                          '17 h6 can produce interference or clearance; fit and residual CN clearance remain blocked',
                          'short stack: add annular outboard stack shims; long stack: add Ø16.8/Ø6.6 shaft-face standoffs under end washers; target 0.05..0.15 end play without bearing preload',
                          'hole true position, cap screw engagement, tool access and thermal gradients need independent review'],
            'fabrication_released':False}


def select_end_shims(measured_stack_mm, measured_shaft_mm):
    """Proposed shim selection from ACTUAL measurements, never a measurement."""
    positive(measured_stack_mm,'measured stack [mm]');positive(measured_shaft_mm,'measured shaft [mm]')
    difference=measured_shaft_mm-measured_stack_mm
    stack_shim=max(0.,difference-.1)
    face_shim=max(0.,.1-difference)
    if stack_shim>.75 or face_shim>.50:raise ValueError('shim adjustment exceeds review envelope; open a deviation')
    return {'annular_stack_shim_mm':stack_shim,'shaft_face_standoff_mm':face_shim,
            'target_end_play_mm':.1,'status':'selection_proposal_requires_inspection_not_measured_evidence'}


def attachment_screens(design, peak_force_N):
    """Nominal, unnotched attachment screens at a stated conditional force."""
    positive(peak_force_N,'peak force [N]');c=design['fabrication_geometry']['cartridge']
    r=c['stop_bar_od']/2;a=c['stop_bar_flat_offset']
    if not 0<a<r:raise ValueError('crossbar flat must lie inside the circular section')
    removed_area=r*r*math.acos(a/r)-a*math.sqrt(r*r-a*a)
    area=math.pi*r*r-removed_area
    centroid=(2/3*(r*r-a*a)**1.5)/area
    primitive=lambda z:(z*math.sqrt(max(0,r*r-z*z))*(2*z*z-r*r)+r**4*math.asin(z/r))/4
    inertia=math.pi*r**4/4-(primitive(r)-primitive(a))-area*centroid**2
    modulus=inertia/max(r-centroid,a+centroid)
    # Supports at the midplanes of the journals, not at the shoulders.
    bar_span=2*c['stop_carrier_frame_y']
    lug_modulus=c['stop_lug_width']*(2*c['stop_lug_root_half_height'])**2/6
    frame_modulus=c['stop_carrier_width_y']*c['stop_carrier_beam']**2/6
    return {'basis_force_N':peak_force_N,'status':'conditional_nominal_screens_not_capacity_approval',
            'crossbar_flat_section_area_mm2':area,'crossbar_flat_section_modulus_mm3':modulus,
            'crossbar_unnotched_bending_MPa':peak_force_N*bar_span/4/modulus,
            'crossbar_journal_bending_MPa':32*(peak_force_N/2)*(c['stop_carrier_width_y']/2)/(math.pi*c['stop_bar_journal']**3),
            'lug_root_unnotched_bending_MPa':peak_force_N*(c['stop_lug_radius']-15)/lug_modulus,
            'frame_unnotched_bending_MPa':peak_force_N*c['stop_lug_radius']/2/frame_modulus,
            'shaft_end_bolt_tension_MPa_at_300N':300/20.1,
            'limitations':['crossbar flat is included analytically; drilled/tapped holes, shoulders and local contact concentrations are NOT included',
                          'lug and C-frame use nominal rectangular sections and conservative full stop couple; not a frame stiffness solution',
                          'carriage plate bending, threaded-foot pullout/prying, cap bending and screw preload/locking require independent calculation/inspection',
                          'no fatigue, ultimate material certification, deflection or physical qualification is supplied']}


def engineering_report(design, scope):
    m=scope['mission_inputs']['ME-MASS-002']['value'];p=scope['mission_inputs']['ME-PAYLOAD-001']['value']
    energy=scope['mission_inputs']['ME-IMPACT-001']['value']*scope['mission_inputs']['ME-SF-001']['value']['ultimate']
    length=design['dummy_member']['effective_length_mm'];c=design['fabrication_geometry']['cartridge']
    rows=[]
    for travel in (1,2,4):
        for fraction in (.5,.75,1):
            r=impact_free_body(design,m,p,length,energy,travel,fraction)
            rows.append({'travel_mm':travel,'area_fraction':fraction,'force_N':r['stop_force_N'],'radial_per_bearing_N':r['maximum_bearing_radial_N'],'s0_without_gravity':4750/r['maximum_bearing_radial_N']})
    two_deg=c['stop_lug_radius']*math.sin(math.radians(2))
    lines=['# A0-R2 engineering audit','', '**CONDITIONAL DESIGN CALCULATIONS — NOT PHYSICAL EVIDENCE.**','',
           'The frame is right handed: X/Y/Z in mm, +Y pitch shaft through S. For bearings at yA/yB, rigid equilibrium gives RBx=(Mz+yA Fx)/span and RBz=(-Mx+yA Fz)/span; RA closes force equilibrium. A carries axial Fy; B floats. Shaft-axis My remains a separate torque reaction, never a radial bearing couple.','',
           'Directional handling applies ±100 N service (±300 N retention) at E=(-317,0,0) from S. The JSON lists all positive X/Y/Z free bodies; negative directions reverse reactions with identical magnitudes in this linear idealization. Gravity and commanded acceleration use the frozen mass at L/2 and payload at L. The bearing axial location, key, hub, yoke feet, carrier and fixture connections require independent stiffness/preload review.','',
           'At a tangent pitch stop, F=E/(a·s), where a is the force-curve area divided by Fpeak·travel. The legacy triangular assumption has a=0.5. Torque is F·R60; angular deceleration is torque/J; the member inertia resultant is alpha·(mL/2+pL). Because the member and integral positive-X lug lie opposite S, the stop and inertia resultants add in the bearing radial free body. Gravity is added conservatively to the peak load in A0_LOAD_CASES. Stop torque flows through the integral hub lug into the crossbars/carrier, while the shaft sees radial bending. Index retention torque uses a separate keyed-shaft free body.','',
           'Service energy 2 J and design energy 6 J are retained separately. No additional factor of three is applied to that peak force. Frozen material yield/ultimate, buckling, joint retention and keyed-insert factors are unchanged; the key is now explicitly checked at 4× service torque. Material lot allowables, fatigue, fastener preload/thread pullout, contact stresses and notches remain unqualified. Rocker weak-axis inertia now uses its 25 mm section width rather than its 70 mm overall length; root shear area uses the 10 mm shoulder rather than the clearance bore.','',
           '## Hypothetical sensitivity, not an evidence-bounded material model','',
           '| Travel mm | Curve area fraction | Stop force N | Radial per bearing N | C0/P0 (gravity omitted here) |','| ---: | ---: | ---: | ---: | ---: |']
    lines += [f"| {r['travel_mm']} | {r['area_fraction']} | {r['force_N']:.3f} | {r['radial_per_bearing_N']:.3f} | {r['s0_without_gravity']:.4f} |" for r in rows]
    lines += ['',f'Nominal first contact is -32/+92 degrees. Four millimeters at R60 would require about {math.degrees(4/c["stop_lug_radius"]):.3f} degrees additional motion. Even moving first contact to the command edge permits only {two_deg:.4f} mm in the two-degree margin. No row in this table establishes usable travel. Resolve the energy/range conflict and measure a real rate-dependent curve; the four-millimeter acceptance criterion stays open.','',
              '## Interfaces and supplier comparison','',
              'SKF 6003-2RSH (C0 3250 N) fails project s0=2 under the revised conditional impact model. SKF 6203-2RSH (C0 4750 N, C 9950 N, 17×40×12) passes that particular arithmetic screen with the assumed 4 mm curve; it is a candidate only. Housing abutment opening 35.00..35.03 is below the catalog maximum35.8; spacer OD21.58..21.62 is within21.2..22; abutment radius≤0.4 is below catalog maximum0.6. Primary sources, dates and exact applicability limits are recorded in a0_supplier_basis.json.','',
              'Worst-case interval stacks use project incoming acceptance windows, not invented SKF manufacturing tolerances. They establish positive nominal retention and nonnegative pocket clearance in those windows. They do not establish mounted internal clearance. The retained 17 h6 shaft can have interference or clearance relative to incoming bore limits; rotating/oscillating inner-ring fit, creep, CN clearance and thermal behavior therefore remain critical holds. Floating-side pocket15 and cap nose1.25 provide clearance in both axial directions; the interval check includes the locating-ring pocket travel as well as center-span mismatch. Select end shims from the measured entire stack: annular stack shims for a short stack, small shaft-face standoffs for a long stack. Neither actual measurement nor shim selection is supplied. Do not tighten away end play by preloading the bearing rings.','',
              '## Structural and article boundary','',
              'A0_R2_ATTACHMENTS.json adds geometry-derived unnotched lug/frame/crossbar screens at the conditional peak force. The crossbar section includes its milled flat but does not include thread/drill stress concentrations. These are not connection qualification. CAD establishes shapes, positive end capture, holes and nominal finite-pose compatibility. It does not qualify the C-cap retaining arc, lug root, stepped carriage, stop frames/crossbars, threaded mounting feet or keyways. Closed-form shaft/key/yoke net-section checks omit local stress concentrations, joint flexibility and preload. Require independent structural/load-path review and supported allowables before manufacture. No FEA or simulation qualification has occurred.','',
              'Legacy scapular stop/plunger and straight tether envelopes are excluded from the issued subassembly, with their original conflicts listed in the exact intersection report. Those open interfaces, friction grounding/coupling, complete rocker/bolt sweep, guards, tool access and the external bench prevent whole-article release. All six load cases and five evidence packages remain unapproved.','']
    return '\n'.join(lines),rows
