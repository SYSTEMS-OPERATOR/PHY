# SOPHY Geometric Canon

Canon version: `SOPHY_GEOMETRY_CANON_VERSION = 1.0.0`

## Definition

SOPHY’s canon is a versioned mathematical identity geometry instantiated at a selected master scale. It is not a population-average body, an attractiveness score, a visual-reference trace, or a collection of plausible measurements.

The executable contract is:

```python
geometry = instantiate_sophy_canon(height_mm, canon_version="1.0.0")
```

Equivalent notation:

```text
geometry = SOPHY_CANON(H, canon_version)
```

Changing `H` scales the same canon. Changing a normalized landmark equation, symmetry law, active proportional binding, or coordinate convention changes SOPHY’s physical identity and requires a new canon revision.

## Authority

- `BODY.md` is the physical authority.
- `skeleton/canon/geometry.py` is the executable realization of the canon revision named by BODY.
- `skeleton/schema/sophy_canonical_geometry.schema.json` is the neutral artifact contract.
- `SOUL.md` supplies symbolic inheritance only. A symbolic principle becomes fabrication truth only after BODY and the executable kernel formalize it.
- Project profiles bind to a canon instance; they do not own identity coordinates.

The bone schema remains authoritative for bone fabrication records. The whole-body canon sits beneath it and does not silently rewrite existing bone modules.

## Independent variables

Canon `1.0.0` has one independent scalar:

| Symbol | Meaning | Constraint |
| --- | --- | --- |
| `H` | Canonical standing height / master scale | Finite, positive, millimetres |

Every exported coordinate is normalized to `H` and then scaled. Material, renderer, mechanism, project, age label, or fabrication method is not an identity variable.

## Coordinate system

The canonical frame is right-handed:

| Axis / plane | Definition |
| --- | --- |
| Origin | `body.plantar_midpoint`, the ground-plane midpoint of the canonical body frame |
| `+x` | Subject’s right |
| `+y` | Subject’s anterior |
| `+z` | Superior |
| Midsagittal plane | `x = 0` |
| Units after instantiation | millimetres |

The neutral reference pose identifier is `SOPHY_NEUTRAL_T_POSE_V1`. Version `1.0.0` binds the global frame and span construction. Joint-centre and anatomical hand/foot coordinates remain deferred rather than guessed.

## Canonical laws in version 1.0.0

### Scale and span

```text
standing_height = H
arm_span = H
```

### Bilateral reflection

Only the canonical right-side definition is authored:

```text
LEFT(P) = mirror_x(RIGHT(P))
mirror_x(x, y, z) = (-x, y, z)
```

No left canonical landmark may be authored independently. A built article may differ, but the difference is stored as `measured_deviation_from_canon` with evidence and a reason.

### Square construction

The square lies in the `XZ` plane:

```text
x ∈ [-H/2, H/2]
z ∈ [0, H]
y = 0
```

It follows the historical height/span relationship but is implemented as a PHY/SOPHY canonical design law.

### Circle construction

Canon `1.0.0` defines the circle inscribed in that square:

```text
center = (0, 0, H/2)
radius = H/2
plane = XZ
```

This is explicitly a PHY/SOPHY canonical design law. It is not attributed to Leonardo and is not a reconstruction of the historical navel-centred spread-pose circle. The anatomical navel is not bound to this centre in version `1.0.0`.

## Historical boundary

Vitruvius, *De architectura*, Book III, Chapter I describes:

- height equal to outstretched arm span;
- a circle centred at the navel in a spread pose;
- face height as `H/10`;
- hand length as `H/10`;
- chin-to-crown head height as `H/8`;
- foot length as `H/6`;
- forearm length as `H/4`;
- breast breadth as `H/4`;
- three equal vertical subdivisions of the face.

Primary reference: [Vitruvius, Book III, Chapter I](https://www.gutenberg.org/files/20239/20239-h/20239-h.htm#Page_72).

These are historical reference measurements, not automatically SOPHY laws. Version `1.0.0` adopts only the height/span square relationship. It neither claims that Vitruvius specified φ nor treats Leonardo’s drawing as a fabrication drawing.

## φ and recursive proportional operators

The constant is named once:

```text
phi = (1 + sqrt(5)) / 2
```

The executable operators are:

```text
golden_major(S) = S / phi
golden_minor(S) = S / phi²
golden_recurse(S, depth, branch) = repeated named major or minor division
```

No anatomical landmark is bound to these operators in canon `1.0.0`. This is deliberate. “Golden face” and “Fibonacci hand” remain symbolic inheritance until specific landmark-to-landmark equations are selected and versioned. The kernel therefore contains no decorative `1.618` multipliers.

## Canonical landmark graph

Every landmark record carries:

- canonical identifier;
- family;
- parent frame;
- bilateral or midline side;
- normalized coordinate in units of `H`;
- instantiated coordinate in millimetres;
- derivation operator and inputs;
- provenance classification;
- tolerance category;
- downstream dependents;
- `mirrored_from` for every left landmark.

Bound graph in version `1.0.0`:

| ID | Family | Side | Normalized coordinate `(x/H, y/H, z/H)` | Derivation |
| --- | --- | --- | --- | --- |
| `body.plantar_midpoint` | lower limb | midline | `(0, 0, 0)` | scale by `H` |
| `body.canonical_center` | body centre | midline | `(0, 0, 1/2)` | scale by `H` |
| `cranial.vertex` | cranial | midline | `(0, 0, 1)` | scale by `H` |
| `construction.arm_span_endpoint.right` | upper limb construction | right | `(1/2, 0, 1/2)` | scale by `H` |
| `construction.arm_span_endpoint.left` | upper limb construction | left | `(-1/2, 0, 1/2)` | reflect the right endpoint |

The span endpoints are construction landmarks. They must not be relabelled as anatomical fingertips until the neutral shoulder/arm/hand graph is selected.

### Planned landmark families

The artifact schema supports the requested families without pretending their coordinates are known:

- cranial and facial;
- cervical and shoulder girdle;
- thoracic, lumbar, and pelvic;
- upper limb and hand;
- lower limb and foot.

Adding identity coordinates to these families requires a canon revision.

## Canonical neutral face architecture

The future neutral face graph will be rooted in a canonical face frame derived from cranial midline landmarks. Candidate identifiers include:

- midline: `face.glabella`, `face.nasion`, `face.pronasale`, `face.subnasale`, `face.labiale_superius`, `face.labiale_inferius`, `face.pogonion`;
- bilateral: `face.orbit_center.right`, `face.mouth_corner.right`, `face.gonion.right`, with left nodes derived only by reflection;
- transverse and vertical construction planes that reference named landmarks rather than beauty-score ratios.

Every chosen facial ratio must name both endpoints and an operator. Concept art and renderer output are not dimensional sources.

## Expression architecture

Expressions are reversible transforms over the neutral face:

```text
canonical_neutral_face
    + expression_deformation(parameters, basis, weight)
    = expressed_face
```

Rules:

1. Zero deformation exactly returns the canonical neutral state.
2. Expression transforms cannot write canonical landmark coordinates.
3. Repeated application must not accumulate identity drift; transforms are evaluated from neutral, not from the previous expression frame.
4. Symmetric expression controls may use mirror-derived transforms. Intentional expressive asymmetry belongs to transient deformation state, not the neutral canon.

Version `1.0.0` prepares this boundary but does not invent facial coordinates or expression bases.

## Canon versus embodiment

The relationship is:

```text
SOPHY_CANON(H, version)
    -> project binds scale and revision
    -> engineering overlay adds materials, envelopes, clearances, and measured deviations
    -> physical article is inspected against canon
```

`EmbodimentOverlay` stores a canon reference and optional `MeasuredLandmarkDeviation` records. It rejects top-level fields that attempt to replace landmarks, dimensions, symmetry, the coordinate system, or canon identity.

Mechanical overlays may define:

- structural thickness and laminate schedules;
- bearings, cartridges, actuator clearance, and cable passages;
- fascia and soft-volume envelopes;
- service panels and inspection access;
- manufacturing tolerances;
- independently calibrated friction, stiffness, sensors, and actuator zeros.

These may alter the realization or measured article without altering canonical coordinates.

## Tolerances

Canonical laws are exact mathematical relationships. Floating-point validation uses a numerical comparison tolerance of:

```text
max(1e-9 mm, H × 1e-12)
```

This tolerance acknowledges representation error; it is not permission for design drift. Manufacturing tolerance and article deviation belong to the embodiment record.

## Validation invariants

`CanonicalGeometryValidator` detects:

- unsupported or mismatched canon revision and law hash;
- `arm_span != H`;
- `standing_height != H`;
- duplicate landmark IDs;
- unsupported derivation operators;
- coordinates inconsistent with normalized coordinates and `H`;
- any midline `x != 0`;
- any left landmark without a right-side mirror source;
- broken mirror dependencies or reflected-coordinate mismatch;
- missing span endpoints;
- square/circle cardinal-point drift.

`CanonicalGeometry.from_dict()` reconstructs the expected geometry from `H` and the explicit version, then rejects any payload that differs. Import does not trust duplicated exported coordinates.

## Deterministic export

Command:

```bash
PYTHONPATH=. bin/export_sophy_canon.py --height-mm 1676.4
```

Outputs:

- `dist/sophy_canonical_geometry.json`;
- `reports/sophy_canon_validation_report.json`.

The JSON is renderer- and fabrication-neutral and includes scale, version, law hash, coordinate system, dimensions, construction geometry, normalized and absolute landmarks, derivations, provenance, and unresolved identity decisions. It contains no timestamp, so identical input and code produce identical bytes.

## Example: SOPHY at 1676.4 mm

```text
H = 1676.4 mm
standing_height = 1676.4 mm
arm_span = 1676.4 mm
body.canonical_center = (0.0, 0.0, 838.2) mm
right span endpoint = (838.2, 0.0, 838.2) mm
left span endpoint = (-838.2, 0.0, 838.2) mm
canonical circle radius = 838.2 mm
```

These are coordinates, not suggestions.

## Revision policy

The following require an explicit new `SOPHY_GEOMETRY_CANON_VERSION` and a changed canon law hash:

- normalized landmark equations;
- active φ or other proportional bindings;
- symmetry or mirror rules;
- coordinate-system orientation or origin;
- canonical neutral-pose identity geometry;
- face, limb, hand, foot, torso, or pelvic ratios;
- construction laws that define identity.

The following do not change the canon when canonical coordinates remain intact:

- new materials;
- bearing, actuator, cable, or panel improvements;
- fabrication processes and tolerances;
- serviceability changes;
- measured deviations on an individual article;
- renderer, CAD, URDF, Blender, or fixture export adapters.

Historical revision labels must never be run through newer equations. Code must explicitly implement any supported old revision.

## Unresolved identity decisions

These decisions remain deliberately open:

1. Whether SOPHY’s anatomical navel coincides with `body.canonical_center`, and therefore whether the historical and canonical circles are related or separate.
2. Which, if any, Vitruvian fractions become SOPHY design law.
3. Which named anatomical intervals, if any, bind to φ or recursive operators.
4. Neutral shoulder, elbow, wrist, hand-tip, hip, knee, ankle, and plantar landmark coordinates.
5. Head/body subdivision and neck/torso/pelvis segmentation.
6. Canonical hand, finger, foot, and toe segmentation.
7. The exact neutral facial landmark graph and any vertical/transverse divisions.
8. Foot separation and depth geometry in the neutral reference pose.

Until decided, these remain unbound. The absence is machine-visible in the export and cannot be mistaken for a zero or average measurement.
