# A0-R2 engineering audit

**CONDITIONAL DESIGN CALCULATIONS — NOT PHYSICAL EVIDENCE.**

The frame is right handed: X/Y/Z in mm, +Y pitch shaft through S. For bearings at yA/yB, rigid equilibrium gives RBx=(Mz+yA Fx)/span and RBz=(-Mx+yA Fz)/span; RA closes force equilibrium. A carries axial Fy; B floats. Shaft-axis My remains a separate torque reaction, never a radial bearing couple.

Directional handling applies ±100 N service (±300 N retention) at E=(-317,0,0) from S. The JSON lists all positive X/Y/Z free bodies; negative directions reverse reactions with identical magnitudes in this linear idealization. Gravity and commanded acceleration use the frozen mass at L/2 and payload at L. The bearing axial location, key, hub, yoke feet, carrier and fixture connections require independent stiffness/preload review.

At a tangent pitch stop, F=E/(a·s), where a is the force-curve area divided by Fpeak·travel. The legacy triangular assumption has a=0.5. Torque is F·R60; angular deceleration is torque/J; the member inertia resultant is alpha·(mL/2+pL). Because the member and integral positive-X lug lie opposite S, the stop and inertia resultants add in the bearing radial free body. Gravity is added conservatively to the peak load in A0_LOAD_CASES. Stop torque flows through the integral hub lug into the crossbars/carrier, while the shaft sees radial bending. Index retention torque uses a separate keyed-shaft free body.

Service energy 2 J and design energy 6 J are retained separately. No additional factor of three is applied to that peak force. Frozen material yield/ultimate, buckling, joint retention and keyed-insert factors are unchanged; the key is now explicitly checked at 4× service torque. Material lot allowables, fatigue, fastener preload/thread pullout, contact stresses and notches remain unqualified. Rocker weak-axis inertia now uses its 25 mm section width rather than its 70 mm overall length; root shear area uses the 10 mm shoulder rather than the clearance bore.

## Hypothetical sensitivity, not an evidence-bounded material model

| Travel mm | Curve area fraction | Stop force N | Radial per bearing N | C0/P0 (gravity omitted here) |
| ---: | ---: | ---: | ---: | ---: |
| 1 | 0.5 | 12000.000 | 7766.562 | 0.6116 |
| 1 | 0.75 | 8000.000 | 5177.708 | 0.9174 |
| 1 | 1 | 6000.000 | 3883.281 | 1.2232 |
| 2 | 0.5 | 6000.000 | 3883.281 | 1.2232 |
| 2 | 0.75 | 4000.000 | 2588.854 | 1.8348 |
| 2 | 1 | 3000.000 | 1941.640 | 2.4464 |
| 4 | 0.5 | 3000.000 | 1941.640 | 2.4464 |
| 4 | 0.75 | 2000.000 | 1294.427 | 3.6696 |
| 4 | 1 | 1500.000 | 970.820 | 4.8928 |

Nominal first contact is -32/+92 degrees. Four millimeters at R60 would require about 3.820 degrees additional motion. Even moving first contact to the command edge permits only 2.0940 mm in the two-degree margin. No row in this table establishes usable travel. Resolve the energy/range conflict and measure a real rate-dependent curve; the four-millimeter acceptance criterion stays open.

## Interfaces and supplier comparison

SKF 6003-2RSH (C0 3250 N) fails project s0=2 under the revised conditional impact model. SKF 6203-2RSH (C0 4750 N, C 9950 N, 17×40×12) passes that particular arithmetic screen with the assumed 4 mm curve; it is a candidate only. Housing abutment opening 35.00..35.03 is below the catalog maximum35.8; spacer OD21.58..21.62 is within21.2..22; abutment radius≤0.4 is below catalog maximum0.6. Primary sources, dates and exact applicability limits are recorded in a0_supplier_basis.json.

Worst-case interval stacks use project incoming acceptance windows, not invented SKF manufacturing tolerances. They establish positive nominal retention and nonnegative pocket clearance in those windows. They do not establish mounted internal clearance. The retained 17 h6 shaft can have interference or clearance relative to incoming bore limits; rotating/oscillating inner-ring fit, creep, CN clearance and thermal behavior therefore remain critical holds. Floating-side pocket15 and cap nose1.25 provide clearance in both axial directions; the interval check includes the locating-ring pocket travel as well as center-span mismatch. Select end shims from the measured entire stack: annular stack shims for a short stack, small shaft-face standoffs for a long stack. Neither actual measurement nor shim selection is supplied. Do not tighten away end play by preloading the bearing rings.

## Structural and article boundary

A0_R2_ATTACHMENTS.json adds geometry-derived unnotched lug/frame/crossbar screens at the conditional peak force. The crossbar section includes its milled flat but does not include thread/drill stress concentrations. These are not connection qualification. CAD establishes shapes, positive end capture, holes and nominal finite-pose compatibility. It does not qualify the C-cap retaining arc, lug root, stepped carriage, stop frames/crossbars, threaded mounting feet or keyways. Closed-form shaft/key/yoke net-section checks omit local stress concentrations, joint flexibility and preload. Require independent structural/load-path review and supported allowables before manufacture. No FEA or simulation qualification has occurred.

Legacy scapular stop/plunger and straight tether envelopes are excluded from the issued subassembly, with their original conflicts listed in the exact intersection report. Those open interfaces, friction grounding/coupling, complete rocker/bolt sweep, guards, tool access and the external bench prevent whole-article release. All six load cases and five evidence packages remain unapproved.
