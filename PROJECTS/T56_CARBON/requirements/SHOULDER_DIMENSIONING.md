# T56 shoulder dimensioning work package

Status: **ENGINEERING REVIEW — NO FABRICATION RELEASE**

The next article is `single_side_thoracic_root_to_dummy_humerus`. The forearm
remains the later distal systems demonstrator. This package implements the next
mapping step after the reference selection merged in PR #92. It does not adopt
new identity landmarks or close the existing G0–G3 gates.

## Decision DR-SHO-001: dummy-member dimensional convention

For the first dimensional study, use a straight, removable dummy member with
collinear end interfaces. This is a project-local study model; bent or eccentric
members require a different model. The dimension resolver explicitly accepts
only this model. No member diameter, wall thickness, stock material, purchased
cartridge, or load rating is selected by this decision.

Use two named mechanical reference points in `FRAME_T56_THORAX` at one recorded
fixture pose:

- **S**: the shoulder cartridge's selected humeral output reference point. For
  intersecting axes this may be their intersection; for offset axes the cartridge
  drawing must identify which output frame origin S denotes. Do not assume a
  ball center or copy the acromion coordinate.
- **E**: the dummy elbow/load-station reference point. This article does not need
  a fabricated biological elbow, but its station must be dimensioned to represent
  the intended load application. It is not the radiale landmark.

For `u = (E - S) / |E - S|`, let `L = |E - S|`. Positive shoulder and elbow
offsets `oS`, `oE` run inward along the member from their respective reference
points to the tube-mouth seats. Positive insertions `iS`, `iE` run from those
seats back into the corresponding cartridge sockets:

```
seat_S = S + oS*u                 seat_E = E - oE*u
cut_S  = seat_S - iS*u            cut_E  = seat_E + iE*u
seat_gap = L - oS - oE
stock_cut_length = seat_gap + iS + iE
```

The calculation requires a positive seat gap, explicit units, source evidence,
and positive tolerance bounds for every input. Point tolerances are Euclidean
position-error bounds in mm; offset/insertion tolerances are axial bounds in mm.
The conservative length bound adds both endpoint bounds and all four axial
bounds. It is a worst-case dimensional stack, not a statistical percentile or
strength allowance. Require a positive seat gap even at its lower bound.

Stock cut length is distinct from `GEO-ARM-001` effective length. The drawing
shows nominal reference, seat, and cut-end points in three orthographic views.
It is a centerline dimension review, **not** a part drawing: no cross-section,
hole pattern, socket detail, fit, or process allowance is implied. Tube cutting
requires a later reviewed production drawing.

## Anatomical evidence to mechanical geometry

| Reference from PR #92 | What it can inform | What it cannot supply |
| --- | --- | --- |
| Acromion–radiale, median 317 mm | External upper-arm comparison | S-to-E distance, humerus bone length, or cut length |
| Biacromial breadth, median 369.5 mm | External shoulder breadth comparison | Cartridge-center spacing or root hardpoints |
| Radiale–stylion, median 244.5 mm | External forearm comparison | Radius/ulna cuts or elbow/wrist cartridge centers |
| Chest breadth/depth | Outer-envelope comparison at the source's measurement pose | Structural housing or collision envelope without a mapping |

Values above cite `../reports/REFERENCE_DIMENSIONS.md`; the endpoint definitions
and limitations in `../references/DIMENSIONING_PROCESS.md` control their use.
Do not sum marginal medians into a T-pose span. When adopting identity landmarks,
author the right side and derive the left by reflection in a reviewed canon
revision. Fixture points in this study are local mechanical inputs and are not
new SOPHY identity coordinates.

## Drawing and measurement closure order

| Register | Required work | Deliverable |
| --- | --- | --- |
| Datum | Realize origin, right-handed axes and root restraints; record drawing revision and tolerances | Fixture datum/interface drawing, EV-ARTICLE-001 |
| SHO-006, ARM-001 | Identify S and E from cartridge/load-station drawings in the same frame and pose | Centerline dimension review from this tool; later reviewed register adoption |
| SHO-001/002 | Place posterior and anterior roots relative to the fixture datum | Toleranced root interface drawing |
| SHO-003/004 | Compare guided carriage and coupled-linkage candidates; calculate mobility and check overconstraint before selection | Mechanism decision, guide path and ordered linkage centers |
| SHO-005/007 | Define required poses, output trajectory, actual joint axes/ranges, stops and passive retention | Kinematic model and EV-ARTICLE-002 decision |
| CLR-001/002/003 | Model neck/outer-form exclusions and tool/connector removal access | Collision and service study, EV-ARTICLE-004 |

All IDs abbreviate the `GEO-` prefix. A trajectory needs a sweep and a motion
model; matching two endpoint poses is insufficient. The centerline review does
not by itself close any row of this table.

## Operating requirements to settle before component sizing

Use the ten inputs already selected by `single_side_article_closure.json`:
arm mass and payload; speed and acceleration; accidental push/pull and impact;
design life; safety factors by failure mode; sound level with measurement
conditions; maintenance handling force. Record proposed design targets as
provisional decisions, never as measured capabilities. Estimate arm mass from a
component budget, then replace estimates with as-built measurements.

For static sizing, include each mass and its own center-of-mass lever arm. A
single humeral length is not the hand-payload lever arm. Dynamic sizing further
requires mass distribution, inertias, trajectories and stopping behavior. Close
all six assigned load cases with numeric criteria before claiming qualification.

## Offline review workflow

The checked-in `shoulder_member_inputs.json` is an explicit open-input record.
It intentionally generates a blocker report and no drawing:

```sh
python -S PROJECTS/T56_CARBON/tools/shoulder_dimensions.py \
  --input PROJECTS/T56_CARBON/requirements/shoulder_member_inputs.json \
  --output-dir /tmp/t56-shoulder-review
```

Exit 2 means unresolved inputs, exit 1 means invalid input, and exit 0 means only
that a **dimensional review** was calculated. Supplied records need local drawing
evidence and tolerance bounds. The output records the input SHA-256, all inputs,
derivation, bounds and three-view SVG. It never writes the geometry registers,
changes the canon, sets fabrication readiness true, or approves evidence packages.
Do not use the synthetic unit-test coordinates as project measurements.

## Release handoff

Before release, the packet still needs the selected mechanism and materials,
cross-sections, socket and fastener details, BOM/SKUs, load analysis, coupon
allowables, inspection points, assembly instructions and raw article test data.
Run the existing single-side strict convergence check; its blocked result must
remain visible until those inputs and evidence have actually been approved.
