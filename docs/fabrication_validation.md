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
The public runtime `mass_kg()` method retains its existing embodiment gate and
fault behavior.

## Boundaries

Passing record checks does not establish shape-specific dimensional completeness,
measured osteometry, valid articular surfaces, complete joint/contact topology,
positive-definite inertia, swept clearance, strength or physical qualification.
Fabrication release still requires the relevant reviewed article evidence.
Review exports remain available separately from this gate.

SOPHY canon 1.0.0, source bone modules, schema, whole-body proxies, A0-R1 and the
passive maquette are unchanged. Individual bone coverage remains 177/206; the
next geometry scope is 22 individual skull identities, then ear/hyoid coverage.

## Validation checkpoint

Base: main `3e47309d52e3e25c3c4e37be6e1a78f448a8e73d`.

- 21 direct unittest tests pass, including invalid-input mutations, a valid
  positive control, registration preservation, strict JSON, repeated reports and
  embodiment-independent record conversion.
- 147 focused unittest tests pass with no skips across record/bone behavior,
  canon, Studio, hands/feet, reference/dimensional tooling, A0 and source components.
- A separate pytest compatibility selection passes 41 tests, including the new
  direct gate tests, both legacy validators, material/mass and bone/field checks.
  These selections overlap and their counts must not be added together.
- The real fabrication CLI, executed outside the repository root with PYTHONPATH,
  exits 1 as expected. Both report files repeat byte-identically and parse as strict
  JSON. The 193-record model reports 157 bones with unknown dimensions, 172 missing
  dataset bindings, and 193 missing joint and mount definitions; readiness is false.
- Full local pytest collection finds 175 tests but cannot import four modules:
  `test_dsar_delete.py` requires `fastapi`; `test_round7_cognition.py`,
  `test_round8_language_social.py` and `test_round9_meta_dist.py` require
  `gymnasium`. No local full-repository or browser pass is claimed. Remote CI
  supplies these dependencies and remains required.

To run the direct gate regressions using only the standard library:

```bash
PYTHONPATH=. python -m unittest discover -s tests -p test_fabrication_validation.py -v
```
