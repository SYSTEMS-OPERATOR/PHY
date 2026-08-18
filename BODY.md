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

## SOPHY Whole-Body Geometry Canon

SOPHY is a deterministic geometric body canon instantiated at a selected master scale, not an anthropometric approximation.

Authoritative implementation and contract:

- kernel: `skeleton/canon/geometry.py`
- artifact schema: `skeleton/schema/sophy_canonical_geometry.schema.json`
- validation: `skeleton/validation/canon_validator.py`
- technical specification: `docs/sophy_geometry_canon.md`
- current identity revision: `SOPHY_GEOMETRY_CANON_VERSION = 1.0.0`

Authority order for physical geometry:

1. this BODY policy;
2. the versioned SOPHY canon kernel;
3. canonical bone records and schemas;
4. project embodiment overlays;
5. measured article deviations.

Project profiles may select `H` and a canon revision. They may not overwrite canonical coordinates, symmetry, or proportional laws. Mechanical allowances and measured deviations must remain separate overlay data.

Canon `1.0.0` fixes:

- one independent scale variable, `H`, in millimetres;
- a right-handed frame with `+x` subject-right, `+y` anterior, `+z` superior;
- the midsagittal plane `x = 0`;
- `standing_height = H` and `arm_span = H`;
- `LEFT(P) = mirror_x(RIGHT(P))` for all bilateral canonical geometry;
- a square spanning `[-H/2, H/2] × [0, H]` in the `XZ` plane;
- an inscribed SOPHY canonical circle centred at `(0, 0, H/2)` with radius `H/2`.

The circle is a PHY/SOPHY design law, not a historical claim about Leonardo. Anatomical navel, face, head, limb, hand, foot, and active phi relationships remain unbound until adopted by an explicit canon revision.

Any change to normalized landmark equations, active proportional bindings, symmetry, coordinate conventions, or neutral identity geometry requires a new canon version. Material, mechanism, tolerance, and service changes that preserve canonical coordinates do not.

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

Whole-body identity invariants are independently enforced by `skeleton/validation/canon_validator.py`, including scale, arm-span equality, midline centring, exact bilateral derivation, dependency integrity, supported operators, and canon law hash.

Validation outputs:
- `reports/validation_report.json`
- `reports/validation_report.md`
