# THE WOODEN ARMATURE BIBLE

Draft source chapter: Front Matter.

## Working subtitle

A PHY-compatible fabrication reference for wooden humanoid armature design, material selection, joint geometry, refined aging, and offline validation.

## Intent

This book specifies a wooden armature as a layered physical system.

It is not only a skeleton model. It is a fabrication reference for shaped components, drilled interfaces, copper/brass hardware, motion thresholds, soft-tension behavior, sensor routing, surface finishing, and long-duration aging.

## Authority model

Universal PHY truth remains in the core repository:

- `BODY.md`
- `MIND.md`
- `SOUL.md`
- `skeleton/base.py`
- `skeleton/schema/bone.schema.json`
- `skeleton/bones/`

Wood-specific and build-specific truth lives under `PROJECTS/`.

Species-specific truth lives under each project overlay:

- `PROJECTS/DEFAULT/`
- `PROJECTS/REDWOOD/`
- `PROJECTS/BRISTLECONE/`
- `PROJECTS/JUNIPER/`
- `PROJECTS/CYPRESS/`

## Core rule

No missing measurement may be invented as fabrication truth.

Unknown values must remain explicit.

## Units

Canonical PHY units:

| Quantity | Unit |
|---|---|
| length | mm |
| mass | kg |
| density | kg/m3 |
| inertia | kg*m2 |
| angle | degrees unless otherwise specified |
| force | N when measured |
| torque | N*m when measured |

## Notation

- `DOF` means degree of freedom.
- `ROM` means range of motion.
- `hard stop` means a physical limit that prevents further motion.
- `soft stop` means a compliant limit using elastic, frictional, or damped resistance.
- `neutral` means the intended resting pose for assembly, measurement, and validation.
- `component` means any buildable physical part, not only anatomical bones.
- `coupon` means a sacrificial test sample used to validate material behavior before final cuts.

## Validation philosophy

A component is not ready because it looks correct.

A component is ready when it can be:

1. sized
2. cut
3. drilled
4. assembled
5. moved through intended DOF
6. checked against thresholds
7. simulated or approximated for mass behavior
8. inspected after material-aging tests
9. repaired or replaced if needed

## Reading diagrams

Diagrams are intentionally elementary and text-first so they remain useful offline.

Example:

```text
HINGE

[ A ]---O---[ B ]
        ^
        rotation axis

DOF: 1 rotational
motion: flexion / extension
limit: min_angle_deg to max_angle_deg
```

Diagrams show principles. Final dimensions belong in component records and cut lists.

## Safety note

This document is a design and fabrication planning reference. Load-bearing, powered, or human-interactive assemblies require local testing, inspection, and conservative failure handling before use.
