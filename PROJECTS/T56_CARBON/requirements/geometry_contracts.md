# T-5.6 fabrication geometry input contract

Status: **INPUT_CONTRACT_ONLY — NO DRAWING RELEASE**

The executable contract is `../tools/geometry_contracts.py`, enforced by both
convergence scopes. It supplies representations, not measurements. All 19
current geometry values remain open. A valid payload is not evidence of fit,
kinematic coverage, collision freedom, strength, or fabrication readiness.

## Authority and variants

`BODY.md` and the selected `SOPHY_CANON(H, canon_version)` govern identity.
The audit instantiates that kernel and checks the profile, mission register,
and geometry register against it. Agreement between copied values alone is
insufficient. Missing profiles and unsupported revisions fail closed.

Scale changes require a consistent project binding. Material and mechanism
variants reuse these engineering input shapes without rewriting identity laws.
This change does not create a new variant or rescale REDWOOD (currently 1650 mm).
Neither population averages nor visual references can close unbound anatomical
landmarks. A dummy article's effective length is an engineering input, not an
implicit canonical humerus length.

## Shared datum

All non-scalar records with supplied values require
`"frame_id": "FRAME_T56_THORAX"`. Coordinates are local engineering coordinates:
X subject-right, Y anterior, Z superior. Signed and zero coordinates are valid.
No canonical shoulder/hip center or thorax-to-canon placement is inferred.

The register's `coordinate_frame` remains dimensionally open. Before either
scope can pass, it needs `dimensioned_datum_status: "locked"`,
`definition_status: "locked"`, `handedness: "right_handed"`, a nonblank
`origin_definition`, `datum_revision`, and `dimensioned_datum_evidence` pointing
to a reviewed fixture/datum drawing. That drawing must realize the origin,
axes, restraints, and toleranced interfaces. The checker verifies declarations,
not the drawing's engineering adequacy. Full-body placement is a separate task;
the single-side fixture does not require inventing a global anatomical origin.

## Value shapes

All numbers must be finite JSON numbers, never booleans or numeric strings.
Nested objects use exactly the named keys, so misspelled/ambiguous geometry is
rejected rather than ignored. Record metadata may include additional evidence.

| IDs | Value shape | Units |
| --- | --- | --- |
| THX-001/002/003, PEL-001, ARM-001 | Positive scalar length | mm |
| THX-004 | Signed scalar reference offset | mm |
| PEL-003/004, SHO-001/002/006 | `[x, y, z]` point | mm |
| PEL-002 | `{child_frame_id, translation_mm: [x,y,z], rpy_deg: [r,p,y]}` | mm_and_deg |
| SHO-003 | Line or polyline guide, below | mm_and_unit_vector |
| SHO-004 | Ordered list of at least two XYZ joint centers | mm |
| SHO-005 | At least two `{pose_id, position_mm: [x,y,z], rpy_deg: [r,p,y]}` rows | mm_and_deg_by_pose |
| SHO-007 | Nonempty `{joint_id, origin_mm: [x,y,z], axis_unit: [x,y,z], range_deg: [min,max]}` rows | unit_vectors_and_deg |
| CLR-001/002/003 | `{min_mm: [x,y,z], max_mm: [x,y,z]}` axis-aligned bounding box | mm |

IDs above omit the common `GEO-` prefix. Transforms map child points into the
declared parent frame: `p_parent = Rz(yaw) Ry(pitch) Rx(roll) p_child + translation`.
Angles are right-handed degrees; no alternative Euler convention is inferred.
Joint axes/origins are expressed in the thorax frame at the documented reference
configuration. Axis count and orthogonality are mechanism decisions, not assumed
to be three orthogonal axes by the validator.

A line guide is `{type: "line", origin_mm: [x,y,z], direction_unit: [x,y,z],
travel_mm: [min,max]}`. A polyline guide is `{type: "polyline", points_mm:
[[x,y,z], ...]}` with at least two points. Polyline segments are straight;
curved guides need an explicit future representation, not a relabeled polyline.

Unit-vector norm must equal one within `1e-9` (numerical comparison only).
Ranges must strictly increase. Adjacent points cannot coincide. Pose and joint
IDs must be nonblank and unique. Bounding boxes must have positive extent on
every axis; they are conservative review envelopes, not detailed collision meshes.
Trajectory samples do not establish interpolation behavior or swept-volume safety.

## Closure metadata

An unknown value stays `null` with status `open`; no fabricated defaults are
inserted. Supplied provisional values are shape-checked but remain blockers.
Locked records additionally require a nonblank traceable `source` and a tolerance
object with positive finite `linear_mm`. Transform, trajectory, and joint records
also require positive finite `angular_deg`. No tolerance magnitudes are prescribed.
These are engineering tolerances, not permission to alter exact canon equations.

All registered geometry IDs remain required. A new shape, unit system, or parameter
requires an explicit contract and regression-test update. Missing records and
`required_for_simulation: false` cannot silently weaken the gate. A single-side
closure profile must include **every** load case assigned to that article, as well
as the exact union of its input dependencies.

## Verification and limits

Run without third-party packages from the repository root:

```sh
python -m unittest discover -s tests -p 'test_t56_geometry_contracts.py' -v
python -m unittest discover -s tests -p 'test_sophy*.py' -v
python -S PROJECTS/T56_CARBON/tools/check_convergence.py --profile single-side --strict
```

The current strict gate must exit **2**, not 0. Malformed inputs exit 1.
Synthetic test numbers are software fixtures only and must never be copied into
build registers. Tests cover each geometry shape, malformed inputs, canon drift,
datum omissions, dependency omissions, and a synthetic complete-packet positive
control. They are software regression evidence, **not** EV-ARTICLE-005 physical
test evidence. No mission/load approval is supplied or implied. Mission value
semantics and numeric load-acceptance schemas still need dedicated validation;
this PR hardens the geometry boundary, not the entire engineering sign-off chain.
