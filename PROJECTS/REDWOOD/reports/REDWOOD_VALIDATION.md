# REDWOOD Validation Report

Status: `MVP_SCAFFOLD_CREATED`

## Scope

This report tracks the REDWOOD project overlay only. It does not validate or alter universal PHY core files.

## Current pass

- Project config exists: yes
- Adult female 21-28 profile exists: yes
- Redwood material profile exists: yes
- Copper material profile exists: yes
- Brass material profile exists: yes
- Hardware catalogs exist: yes
- Bone guide template exists: yes
- Fabrication class templates exist: yes
- Starter femur guide exists: yes
- Initial cut lists exist: yes

## Known failures / incomplete data

- Most bones do not yet have generated REDWOOD bone guides.
- Most bones do not yet have project-local blank dimensions.
- Joint centers are not yet generated from project joint modules.
- Drill schedule is demonstrator-only for `BONE_FEMUR_L`.
- Hardware items are MVP families, not final purchased SKUs.
- Material strength fields need local references or verified material datasheets.
- Anthropometric targets are MVP project targets and need reference upgrades.
- Simulation mass/inertia outputs are not yet generated.

## Next required pass

1. Add a project overlay loader or generator that reads PHY bone records and REDWOOD templates.
2. Generate long-bone guides for femur, tibia, fibula, humerus, radius, ulna, and clavicle.
3. Normalize parent/child names into project-local connection IDs.
4. Add shoulder, hip, knee, elbow, wrist, ankle, hand, foot, spine, and rib module files.
5. Replace placeholder material constants with referenced local-source records.
6. Generate complete cut lists and drill schedule.
7. Generate simulation mass properties and URDF-like project artifact.
