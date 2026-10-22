# T56 A0 load-case calculation report

Status: **CLOSED-FORM ANALYSIS COMPLETE — PHYSICAL VALIDATION OPEN**

These are deterministic design checks, not measured capability, FEA, or test evidence.

| Case | Key calculated result | Physical disposition |
| --- | --- | --- |
| LC-SHO-001 | service_gravity_moment_Nm = 5.44024 | test record open |
| LC-SHO-002 | ordinary_stop_energy_J = 0.00688741 | test record open |
| LC-SHO-003 | retention_design_moment_Nm = 95.1 | test record open |
| LC-SHO-004 | shaft_yield_margin_x = 2.39749 | test record open |
| LC-SHO-005 | degraded_single_fault_catch_energy_J = 1.40804 | test record open |
| LC-MNT-001 | retention_handling_force_N = 300 | test record open |

## Method limits

- closed-form rigid-joint checks do not replace FEA, coupon data, or article test
- nominal mass is conservatively represented at half-span and payload at the load station
- linear stop force assumes 4 mm usable pad travel and triangular force-displacement response; this assumption is not accepted until a compression curve and first-article stack verify it
- bearing load sharing, friction, damping, backlash, acoustic output, temperature and fatigue remain physical measurements
- key, pin, plate and fastener checks use nominal load distribution without notch, preload-loss or fatigue credit
- no calculated result is a measured capability
- R2 explicitly separates Y-axis torque from bearing bending couples. Rigid-body impact inertia is included; rotor mass distribution remains the frozen two-lump approximation.
- Peak impact and 3x directional retention are separate free bodies, not simultaneous invented combinations.
- Integral lug bypasses shaft/key for stop torque. Impact shaft check uses radial midspan bending; keyed index retention is checked separately.
- Static equivalent bearing load max(Fr,0.6Fr+0.5Fa) is a catalog screen; CN clearance, fits, creep, oscillatory wear and life remain unapproved.
- Stop contact is nominally at -32/+92 degrees; 4 mm tangent compression would overrun by about 3.8 degrees. No impact qualification or compliance credit is granted.

## Shop-review release blockers

- bearing static screen is CONDITIONAL on stop curve, selected SKF variant, fit and internal-clearance approval; not a bearing qualification
- 4 mm stop travel and force-displacement response require supplier/coupon verification before impact testing; that compression overruns the nominal hard-stop range by 3.82 degrees at R60
- combined-load calculations remain closed-form screens and require independent load-path review

Numeric acceptance criteria are recorded in `load_case_register.json`. Passing the arithmetic does not approve a load case; approval requires the corresponding signed bench record.
