# BONE_FEMUR_L — Left Femur

## Purpose

Starter guide for the left femur as a REDWOOD long-bone reference. This file demonstrates the intended bone-by-bone shop format and should be regenerated or refined once project tooling reads PHY canonical bone records directly.

## Finished target envelope

- Finished length: `420 mm`
- Finished width: `40 mm` MVP shaft/joint envelope placeholder
- Finished thickness: `40 mm` MVP shaft/joint envelope placeholder
- Target side: `left`
- Bone class: `long_bone`

## Rough redwood blank

- Blank length: `430 mm`
- Blank width: `44 mm`
- Blank thickness: `44 mm`
- Grain axis: `proximal_to_distal`
- Allowance notes: `10 mm length allowance, 4 mm width/thickness allowance from long_bone_template.json`

## Carving envelope

- Preserve extra material around the hip head/neck and knee condyle zones until drilling and trial assembly are complete.
- Curvature may be shaped by eye inside the finished envelope.
- Do not reduce any joint-end section below the minimum edge-distance rule.
- Mark a centerline along the grain before rough shaping.

## Joint centers

| End | Joint center | Hardware group | Drill target | Notes |
| --- | --- | --- | --- | --- |
| Proximal | `TBD: hip center from pelvis/hip module` | `large_primary` | `12 mm bushing OD placeholder` | Hip interface needs spherical/hinged module design. |
| Distal | `TBD: knee center from tibia/knee module` | `large_primary` | `12 mm bushing OD placeholder` | Knee axis must align with tibia guide. |

## Connections

- Parent: `BONE_HIP_L / pelvis hip module`
- Children: `BONE_TIBIA_L`, `BONE_FIBULA_L via knee module`
- Adjacent guides: `BONE_HIP_L.md`, `BONE_TIBIA_L.md`, `BONE_PATELLA_L.md`

## Hardware

- Pins: `COPPER_PIN_8MM`
- Bushings: `BRASS_BUSHING_8MM_ID_12MM_OD`
- Washers/spacers: `BRASS_WASHER_8MM`, final spacer stack TBD
- Hinge/plate family: `BRASS_SINGLE_AXIS_HINGE_LARGE` for knee; hip module TBD

## Drill schedule

| Operation | Diameter | Depth | Reference face | Notes |
| --- | --- | --- | --- | --- |
| `FEMUR_L_HIP_BUSHING_PILOT` | `TBD` | `TBD` | `proximal end centerline` | Validate against hip module before drilling final diameter. |
| `FEMUR_L_KNEE_BUSHING_PILOT` | `TBD` | `TBD` | `distal end centerline` | Must align with tibia knee axis. |

## Simulation fields

- Approximate mass: `TBD from redwood density and carved/blank volume`
- Center of mass: `TBD; initial estimate at 45-50% length from proximal end`
- Joint axis: `hip TBD, knee medial-lateral`
- Joint limits: `TBD from project joint module`

## Unknowns / required follow-up

- Need exact hip module geometry.
- Need exact knee hinge axis and spacing.
- Need measured or referenced femur width/thickness envelope for desired adult female profile.
- Need final pin length and spacer stack.
- Need mass/inertia calculation after blank and carved geometry are chosen.

## Fabrication status

- Status: `MVP_STARTER_GUIDE_NOT_FABRICATION_READY`
- Validation notes: `Provides length and blank planning only; joint centers, drill depths, and final hardware stack remain incomplete.`
