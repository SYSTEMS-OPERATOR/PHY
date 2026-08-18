# SOPHY Geometry Authority Audit

Status: current-state audit and migration record for canon `1.0.0`.

This audit distinguishes the repository state found before the geometric canon pass from the authority added by the pass. It does not elevate legacy measurements by repetition.

## Authority map

| Layer | Current role | Geometry authority before this pass | Classification / consequence |
| --- | --- | --- | --- |
| `BODY.md` | Fabrication-critical truth | Declared schema, units, material assumptions, tolerances, and physical validation; it did not define whole-body scale, coordinates, or identity landmarks | Universal authority, previously incomplete for whole-body identity |
| `MIND.md` | Runtime/orchestration | Declared deterministic assembly, validation, and export behavior | Universal runtime authority; no whole-body canon command existed |
| `SOUL.md` | Symbolic inheritance | Named arm-span/height, navel, golden face, and Fibonacci hand ideas while explicitly declaring them non-operational | Descriptive only; not fabrication truth |
| `skeleton/base.py` | `BoneSpec` record | Canonicalized bone records and units; its use of “canonical” meant record format, not SOPHY identity geometry | Universal bone-record contract, not a body canon |
| `skeleton/schema/bone.schema.json` | Bone JSON contract | Required fabrication fields but no body scale, coordinate system, symmetry derivation, landmark graph, or canon revision | Universal record schema |
| `skeleton/bones/` | Per-bone modules | Authored dimensions and primitive geometry independently for every module | Legacy/provisional physical inputs |
| `skeleton/datasets/female_21_baseline.json` | Runtime metric overlay | Automatically overwrote matching bone dimensions by bone name | Anthropometric-style legacy/provisional data without sufficient provenance |
| `geometry/geometry_agent.py` | Primitive geometry | Derived boxes/cylinders, volume, mass, centre of mass, and inertia from local bone dimensions | Derived mechanical approximation, not identity geometry |
| `skeleton/validation/validator_agent.py` | Fabrication validator | Checked record completeness, units, local dimensions, materials, references, and mass | No whole-body, scale, symmetry, dependency, or canon-revision validation |
| `skeleton/exporters/exporter_agent.py` | Fabrication exporter | Exported bone records, TF-like origins, URDF-like links, BOM, and tables | No canonical body-scale artifact or canon law metadata |
| `PROJECTS/REDWOOD/` | Redwood embodiment | Held a 1650 mm profile, anthropometric MVP lengths/widths, fabrication allowances, and material decisions | Project-local; geometry profile was legacy/provisional except `arm_span = H` |
| `PROJECTS/T56_CARBON/` | Carbon embodiment | Held `H = 1676.4 mm`, equal arm span, mostly unresolved anatomical dimensions, and mechanical architecture | Project-local; scale is an instance input, packaging is mechanical |
| `references/` | Traceability | Contained minimal bone references, not provenance for the body profiles | Insufficient to promote profile values into canon |
| `tests/` | Regression | Included unit, dataset, mass, and signal tests | `test_broadcast_symmetry.py` tested equal voltage broadcast, not geometric symmetry |

## What physically defined geometry before this pass

The effective runtime dimensions came from two overlapping sources:

1. Each file under `skeleton/bones/` constructed an independent `BoneSpec` with local dimensions and a local geometry dictionary.
2. `skeleton.bones.load_bones()` then applied `female_21_baseline.json` by shared bone name, overwriting matching `BoneSpec.dimensions` values.

The dataset overlay does not update the pre-authored `geometry` dictionary. This creates stale dual truth. Examples include:

| Bone | Module dimensions | Dataset-applied dimensions | Stale module geometry risk |
| --- | --- | --- | --- |
| Femur | 480 × 28 × 28 mm | 420 × 40 × 40 mm | Geometry retains module values while dimensions change |
| Humerus | 350 × 25 × 25 mm | 310 × 30 × 30 mm | Same conflict |
| Tibia | 400 × 25 × 25 mm | 375 × 35 × 35 mm | Same conflict |
| Fibula | 400 mm length | 355 mm length | Same conflict |
| Hip bone / scapula | unresolved in module | populated by dataset | Dataset becomes accidental authority |

These numbers are not promoted into SOPHY canon. They remain bone/anatomical engineering references pending migration.

## Bilateral representation audit

The repository contains:

- 193 bone modules;
- 80 left modules and 80 right modules;
- 80 complete left/right filename pairs;
- 33 midline or unpaired modules.

All 80 paired modules had equal authored `name`, `location`, `dimensions`, and `geometry` values at audit time. However, each left and right value lived in a different source file. Equality was duplicated state, not a derivation. The shared-name dataset loader also happened to apply equal values to both sides.

The migration target is therefore not “make the numbers equal.” It is “author one canonical side and derive the other with `mirror_x`.” Existing bone modules remain legacy inputs until a later bone-by-bone migration can preserve their interfaces safely.

## Proportional assumption classification

| Existing assumption | Found in | Classification | Migration |
| --- | --- | --- | --- |
| Outstretched arm span equals standing height | `SOUL.md`, REDWOOD rules/profile, T56 profile | Canonical | Implement once as `arm_span = H`; profiles reference it instead of owning it |
| Initial left/right equality | Bone pairs and project profiles | Canonical intent, not previously enforced | Implement `LEFT(P) = mirror_x(RIGHT(P))`; keep mechanical calibration separate |
| Master project height | REDWOOD and T56 profiles | Canon instance input | Bind project to `canon_version + H`; height does not change the law set |
| Face golden segments | `SOUL.md` | Symbolic inheritance | No physical binding until BODY adopts named landmark equations |
| Hands/fingers as Fibonacci | `SOUL.md` | Symbolic inheritance | No physical binding; no scattered multipliers permitted |
| Navel as centre | `SOUL.md`; historical Vitruvian description | Historical/symbolic reference | Anatomical navel coordinate remains an explicit identity decision |
| Historical face, hand, head, foot, forearm, and breast fractions | Vitruvius Book III, Chapter I | Historical comparison | Recorded in canon documentation; not active SOPHY laws |
| REDWOOD major widths and primary lengths | `adult_female_21_28.json` | Legacy/provisional anthropometric MVP | Retain with explicit non-canonical status until dependent build guides migrate |
| `female_21_baseline` bone metrics | Skeleton dataset | Legacy/provisional anatomical reference | Stop treating as identity; retain for comparison and bone engineering |
| T56 “adult capable” / “avoid exaggerated display anatomy” | T56 profile | Legacy design intent | Must not select identity coordinates; move to embodiment/contact requirements |
| Hand-carved or fit-calibrated asymmetry | REDWOOD and T56 profiles | Mechanical/article deviation | Record as measured deviation from canon, never as canon mutation |
| REDWOOD blank allowances | REDWOOD proportion rules | Mechanical derived | Remain project-local |
| Pin edge distance, grain direction, bushings | REDWOOD rules | Mechanical | Remain project-local |
| Primitive volume, mass, and inertia | `geometry_agent.py`, `BoneSpec` | Derived mechanical approximation | Remain outside identity kernel |
| T56 clearances, travel, bearing, actuator, and service envelopes | T56 components | Mechanical | Express around or relative to canonical frames |

## Explicit mathematics found before this pass

Existing executable or inspectable relationships were limited to:

- `arm_span_mm = height_mm` in REDWOOD rules and equal stored values in both profiles;
- centimetre-to-millimetre normalization in `BoneSpec`;
- rectangular/cylindrical volume and inertia calculations;
- REDWOOD blank allowances and minimum pin edge-distance rules.

There was no executable whole-body coordinate system, landmark dependency graph, mirror operator, geometric similarity check, or canon identity revision.

## Contradictions and legacy hazards

1. `BoneSpec` called its record canonical while no SOPHY whole-body canon existed.
2. `female_21_baseline` silently overrode module dimensions during normal loading.
3. Dataset-applied dimensions could disagree with the untouched geometry dictionary.
4. Left and right modules were separately editable despite matching at audit time.
5. REDWOOD allowed hand-carved asymmetry after blank layout without requiring deviation records.
6. T56 allowed calibrated asymmetry after fit testing without distinguishing canonical coordinates from mechanical calibration.
7. T56 component language correctly required independent calibration in places, but did not always state that this calibration must not redefine identity geometry.
8. The root fabrication validator could not detect arm-span drift, midsagittal drift, independent left coordinates, or project overwrite.
9. The canonical skeleton export lacked `H`, a whole-body canon revision, landmark derivations, and symmetry metadata.
10. SOUL’s golden/Fibonacci language had no provenance or executable definition and therefore could not safely become fabrication truth.

## Migration dependencies

### REDWOOD

`bin/generate_redwood_bone_specs.py` consumes `primary_lengths_mm` directly, and generated guides and cut lists cite `adult_female_21_28.json`. Those measurements cannot simply disappear without breaking the build packet.

Migration sequence:

1. Mark the profile and every legacy value explicitly non-canonical.
2. Bind the project to `SOPHY_CANON(1650, 1.0.0)` and derive arm span from `H`.
3. Keep current lengths/widths only as a named legacy engineering comparison layer.
4. Add canonical anatomical landmarks and segment equations in a future canon revision after identity decisions.
5. Update the generator to resolve canonical dimensions first and use a legacy fallback only when a required landmark remains unbound.
6. Store carving differences as article metrology deltas.

### T56_CARBON

T56 already leaves most dimensions unresolved and is therefore easier to bind cleanly:

1. Bind to `SOPHY_CANON(1676.4, 1.0.0)`.
2. Derive arm span from `H`.
3. Treat the project thoracic frame as a transform from `SOPHY_CANON`, not a competing global frame.
4. Preserve independent stiffness, friction, sensor-zero, and actuator calibration as mechanical state.
5. Require any physical left/right coordinate difference to be a measured deviation record.
6. Populate project envelopes only after the relevant canonical landmarks exist.

## Audit conclusion

Before this pass, PHY was a deterministic bone-record pipeline with project profiles, not a deterministic SOPHY body canon. The new kernel is deliberately narrow: it establishes the authority boundary and the laws that are already defensible without laundering anthropometric guesses into identity. Future landmark equations can now be added only through an explicit canon revision.
