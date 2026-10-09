# Canonical fabrication validation

The canonical record gate is `skeleton.validation.validator_agent.ValidatorAgent`.
The older `validators/` and `skvalidators/` dataset checks remain compatibility
APIs. Direct canonical regressions live in `tests/test_fabrication_validation.py`;
both Python-package CI (3.11) and Studio CI (3.12) run them.

## Record checks and reporting

Run the offline gate with `PYTHONPATH=. python bin/validate_fabrication.py`.
Exit 0 means record checks pass; exit 1 means the report contains blockers.
Reports are `reports/validation_report.json` and `reports/validation_report.md`.
Every nonempty issue category contributes to `summary.failed_checks` and appears
by name in `export_readiness.issues`. JSON reports reject nonfinite JSON numbers;
invalid values appear as diagnostic strings. Bone order is stable by identity.

The gate checks:

- nonempty input, identity/text fields, typed objects and arrays;
- supplied dimension values: finite positive numbers, with explicit missing/unknown
  blockers for empty dictionaries and `None` entries; strings and booleans are invalid;
- all four canonical units, including `kg*m^2` inertia;
- finite positive density and finite mass in the existing `(0, 100] kg` policy range;
- nonempty joint/mount objects, reference/source/note strings, nonnegative finite
  tolerances, and nonfinite nested geometry/interface/physics values;
- typed parent/child references and source dataset-key membership when a dataset
  is attached;
- conversion failures and normalization overflow as structured blockers.

A valid single-part packet is allowed; this generic gate does not require a
206-bone whole body. Empty input fails. `None` measurements remain unknown; the
validator does not fill them. Invalid conversions do not prevent other bones
from being inspected.

## Loading, datasets and canonical geometry

`load_bones()` discovers definitions in filename order and executes each in a
fresh module namespace. It does not reuse or reload the mutable `bone` singleton
used by legacy module wrappers. Loaded bones, their nested data and their dataset
tables are independent across loads and between bones. An already-mutated legacy
singleton cannot become a new field's source template.

`BoneSpec` retains private copies of its construction-time dimensions, material,
geometry and metric bindings. `apply_dataset()` replaces an overlay: it restores
those source values, copies the selected dataset, clears the previous binding,
then applies available metrics. An absent metric falls back to the source value;
an explicit `None` overrides it with an unknown. A missing bone binding does not
retain measurements, mass metadata or density from an earlier dataset. A supplied
dataset dimension replaces source aliases for that dimension. Dataset application
also invalidates cached runtime geometry; simulation callers must recompute it.

Length keys ending in `_mm`, `_cm` or `_m` normalize to `_mm`, including unknown
values. Conflicting aliases raise a conversion error that the validator reports
as a structured blocker. Known/unknown aliases conflict; two unknowns agree.
Numerically equivalent aliases use a tolerance of `1e-12` relative or `1e-9` mm
absolute and prefer mm, then cm, then m, independent of dictionary order.
Numeric strings and booleans are not coerced into lengths.

Canonical geometry uses the construction-time source shape and current selected
dimensions. The supported shapes are `box` and `plate`, with `length_mm`,
`width_mm` and `thickness_mm`; stored source sizes do not override selected
dimensions. A complete primitive has `status: dimensional_estimate` and
`dimension_source: dimensions`. This status identifies an approximation, not
reviewed anatomical morphology. The default femur therefore exports 420 x 40 x
40 mm in both dimensions and geometry; its original 48 x 2.8 x 2.8 cm source
definition remains intact.

Unknown values remain unknown; missing or invalid dimensions produce unresolved geometry.
Missing source geometry, conflicting shape/type labels, meshes and other shapes
(including the runtime cylinder approximation) produce `status: unresolved` with
a reason, and block the canonical gate. These representations are not converted
into fabrication primitives by guessing. Declared source origins normalize from
explicit mm/cm/m coordinates to `origin_mm`; conflicting origin aliases fail.
Other source geometry attributes and runtime COM, vertex, face and inertia caches
are not copied into the canonical primitive. The mutable `bone.geometry` field
remains available to legacy runtime callers, but does not change the source shape
selected at construction. Construct a new definition to adopt a different shape.
Project display proxies remain a separate geometry pipeline.

Exported records own their nested containers; changing a returned record cannot
alter a bone, a source definition or a later export. The direct regression suite
checks all seven canonical export files for byte-identical output from a fresh
load after another field's material, geometry, signals, links and faults change.

## Registration and mass behavior

`SkeletonField.register()` rejects invalid/duplicate `domain_id` and `unique_id`
before insertion, preserving the original object. Re-registering even the same
object is an explicit duplicate error. Display names may repeat for bilateral
bones. The validator also detects identity collisions introduced by later mutation.

`BoneSpec.to_fabrication_record()` preserves an explicitly supplied mass, including
an explicit unknown `None`. When mass is omitted, it uses the existing box-volume
times density estimate through a pure helper. This is a dimensional estimate,
not measured mass. Record serialization and validation no longer depend on
virtual/physical runtime embodiment or append virtual-state faults.
`physics.mass_provenance` labels a fallback as `estimated` with method
`box_volume_times_density`, or `unknown` if dimensions are unknown. Explicit
`physics.mass_kg` values are labelled `supplied` or `unknown`; caller-provided
provenance for an explicit mass is preserved. Dataset `material.mass_g` remains
legacy metadata and is not silently substituted for the canonical estimate.
The public runtime `mass_kg()` method retains its existing embodiment gate and
fault behavior.

## Boundaries

Passing record checks does not establish anatomical dimensional completeness,
measured osteometry, valid articular surfaces, complete joint/contact topology,
positive-definite inertia, swept clearance, strength or physical qualification.
Fabrication release still requires the relevant reviewed article evidence.
Review exports remain available separately from this gate.

SOPHY canon 1.0.0, source bone modules, schema, whole-body proxies, A0-R1 and the
passive maquette are unchanged.

## Validation checkpoint

Base: main `de1d2c8e099d0ca7d05f02808315e0e0572a1cb3`.

- 39 direct unittest tests pass, including the real femur, legacy-singleton
  contamination, nested isolation, dataset replacement, equivalent/conflicting
  aliases, unknown values, explicit origin units and pure record conversion.
- All seven export files repeat byte-identically from a fresh field after another
  field's runtime and material state changes.
- The real validation, export and assembly CLIs run outside the repository root
  with PYTHONPATH. Validation exits 1 as expected: 193 records, eight blocker
  categories, 157 bones with unknown dimensions and readiness false. Exports have
  no geometry keys ending in `_cm` or `_m`; the femur's dimensions and primitive
  sizes agree. Export/assembly orchestration remains a separate roadmap scope.
- Full local pytest passes with optional PyBullet modules and Docker checks
  skipped where those dependencies are unavailable. The existing Python and
  Studio CI workflows both include the direct canonical regressions.

To run the direct gate regressions using only the standard library:

```bash
PYTHONPATH=. python -m unittest discover -s tests -p test_fabrication_validation.py -v
```
