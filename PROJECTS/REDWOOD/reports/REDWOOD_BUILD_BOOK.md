# REDWOOD Build Book

Status: `MVP_SCAFFOLD_CREATED`

This build book is the project-local offline packet for the REDWOOD armature. It is intentionally incomplete at scaffold time, but defines the exact sections required for a complete offline build.

## 1. Project identity

REDWOOD is a PHY-compatible project overlay for an adult female redwood armature using copper and brass hardware.

## 2. Build doctrine

Measure twice, cut once.

- Use PHY for universal skeleton records.
- Use REDWOOD for project-specific material, proportion, hardware, cut, drill, and assembly decisions.
- Do not treat poetic or symbolic continuity as fabrication truth.
- Do not final-cut any part marked `TBD` or `MVP_STARTER_GUIDE_NOT_FABRICATION_READY`.

## 3. Materials

- Primary structure: `materials/redwood.json`
- Pins/accent hardware: `materials/copper.json`
- Bushings/plates/washers/spacers: `materials/brass.json`
- Finish candidates: `materials/finish_options.json`

## 4. Hardware catalogs

- `hardware/pin_catalog.json`
- `hardware/bushing_catalog.json`
- `hardware/hinge_catalog.json`
- `hardware/washer_spacer_catalog.json`

## 5. Proportion profile

- `profiles/adult_female_21_28.json`
- `profiles/proportion_rules.json`

## 6. Bone guides

Bone guides live in `bone_guides/`.

Current starter guide:

- `bone_guides/BONE_FEMUR_L.md`

Required next guides:

- major long bones
- pelvis/hip modules
- shoulder suspension module
- spine and rib modules
- hand/foot simplified MVP modules

## 7. Cut lists

- `cutlists/redwood_cut_list.csv`
- `cutlists/hardware_cut_list.csv`
- `cutlists/drill_schedule.csv`

## 8. Simulation outputs

Simulation artifacts will live in `simulation/` once generated.

Required future artifacts:

- `simulation/redwood_armature_urdf_like.json`
- `simulation/joint_limits.json`
- `simulation/mass_properties.json`
- `simulation/collision_primitives.json`

## 9. Current hard stop

This scaffold is not yet a complete fabrication packet.

Only `BONE_FEMUR_L` has a starter guide, and it still contains TBDs for joint modules, drill depths, joint centers, and final hardware lengths.

## 10. Next build step

Create REDWOOD project tooling that reads universal PHY records, applies REDWOOD templates, and generates per-bone guides plus project cut lists without modifying PHY core.
