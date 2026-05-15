# {{bone_id}} — {{bone_name}}

## Purpose

This guide converts a PHY-compatible bone record into shop-floor guidance for the REDWOOD armature.

## Finished target envelope

- Finished length: `{{finished_length_mm}} mm`
- Finished width: `{{finished_width_mm}} mm`
- Finished thickness: `{{finished_thickness_mm}} mm`
- Target side: `{{side}}`
- Bone class: `{{bone_class}}`

## Rough redwood blank

- Blank length: `{{blank_length_mm}} mm`
- Blank width: `{{blank_width_mm}} mm`
- Blank thickness: `{{blank_thickness_mm}} mm`
- Grain axis: `{{grain_axis}}`
- Allowance notes: `{{allowance_notes}}`

## Carving envelope

- Preserve extra material around all joint centers until drilling and trial assembly are complete.
- Curvature may be shaped by eye inside the finished envelope.
- Do not reduce any joint-end section below the minimum edge-distance rule.

## Joint centers

| End | Joint center | Hardware group | Drill target | Notes |
| --- | --- | --- | --- | --- |
| Proximal | `{{proximal_joint_center_mm}}` | `{{proximal_hardware_group}}` | `{{proximal_drill_target_mm}}` | `{{proximal_notes}}` |
| Distal | `{{distal_joint_center_mm}}` | `{{distal_hardware_group}}` | `{{distal_drill_target_mm}}` | `{{distal_notes}}` |

## Connections

- Parent: `{{parent_connection}}`
- Children: `{{child_connections}}`
- Adjacent guides: `{{adjacent_guides}}`

## Hardware

- Pins: `{{pins}}`
- Bushings: `{{bushings}}`
- Washers/spacers: `{{washers_spacers}}`
- Hinge/plate family: `{{hinge_family}}`

## Drill schedule

| Operation | Diameter | Depth | Reference face | Notes |
| --- | --- | --- | --- | --- |
| `{{operation_id}}` | `{{diameter_mm}} mm` | `{{depth_mm}} mm` | `{{reference_face}}` | `{{operation_notes}}` |

## Simulation fields

- Approximate mass: `{{mass_kg}} kg`
- Center of mass: `{{center_of_mass_mm}}`
- Joint axis: `{{joint_axis}}`
- Joint limits: `{{joint_limits_deg}}`

## Unknowns / required follow-up

- `{{unknowns}}`

## Fabrication status

- Status: `{{fabrication_status}}`
- Validation notes: `{{validation_notes}}`
