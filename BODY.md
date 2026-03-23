# BODY.md

Authoritative physical specification for PHY fabrication data.

## Scope
`BODY.md` governs fabrication-critical truth:
- canonical bone schema and required fields
- units and conversion rules
- material + physics assumptions
- geometry contracts, joint interfaces, and mount points
- manufacturing notes and tolerances
- physical validation requirements

If `BODY.md` conflicts with symbolic language, `BODY.md` wins.

## Canonical Bone Schema
Authoritative schema: `skeleton/schema/bone.schema.json`.

Required fields per bone record:
- `name`
- `latin_name`
- `bone_type`
- `region`
- `dimensions`
- `units`
- `geometry`
- `material`
- `physics`
- `connections`
- `joint_interfaces`
- `mount_points`
- `manufacturing_notes`
- `tolerance`
- `references`
- `revision`
- `source_ids`

## Unit Conventions
Canonical units are fixed:
- geometry dimensions: **millimeters (mm)**
- mass: **kilograms (kg)**
- density: **kg/m^3**
- inertia: **kg·m^2**

Conversion policy:
- source modules may contain `_cm` values.
- conversion to canonical mm happens in one explicit layer: `BoneSpec.to_fabrication_record()`.
- exported artifacts must be canonical-unit outputs.

## Materials and Physics
- Material density is required.
- Mass plausibility check range (current policy): `(0, 100] kg` per bone.
- Missing density or non-physical values fail fabrication validation.

## Joint Interfaces + Mount Points
- Every fabrication-facing record should provide `joint_interfaces` and `mount_points`.
- If absent, validation flags record as not fabrication-ready.

## Manufacturing Assumptions
- Unknown measurements are not invented.
- Missing dimensions/references are reported as explicit gaps.
- Tolerances default to `{ "default_mm": 0.5 }` unless bone-specific values are supplied.

## Validation Rules (Physical)
Validation categories implemented by `skeleton/validation/validator_agent.py`:
- required field completeness
- duplicate bone IDs
- parent/child reference validity
- impossible geometry values
- unit consistency
- material completeness
- joint/mount completeness
- reference completeness
- mass plausibility
- export readiness

Validation outputs:
- `reports/validation_report.json`
- `reports/validation_report.md`

