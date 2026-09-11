# Shoulder dimensioning verification

Base: PR #92 merged at `0d3a9f0cf37248a623ecc11f9be9decd36f7ba12`.

## Implemented

- Reconciled the shoulder convergence article and later forearm demonstrator.
- Recorded DR-SHO-001, the straight collinear dummy-member study convention.
- Mapped anatomical reference endpoints to the still-required mechanical datums.
- Added deterministic length, insertion/offset and worst-case tolerance resolution.
- Added three-view centerline SVG review output for supplied dimensions.
- Committed the current open-input blocker report under `shoulder_dimension_review/`.

## Verification

52 focused unittest tests passed:

| Suite | Tests |
| --- | ---: |
| test_shoulder_dimensions.py | 9 |
| test_reference_dimensions.py | 12 |
| test_t56_geometry_contracts.py | 18 |
| test_sophy*.py | 13 |

The new suite checks a 3–4–5 vector calculation, socket offset signs, worst-case
bounds, translation invariance, malformed/unsupported inputs, open evidence,
CLI determinism, and removal of stale review drawings when inputs become blocked.
Nonfinite JSON exits cleanly. Test dimensions are synthetic software fixtures.
An SVG rendered from those fixtures was inspected through Inkscape; the three
views, coordinate tables, lengths and review-only labels were legible.

The checked-in open input exits 2 and produces no SVG. The existing strict
single-side convergence audit also exits 2, with schema_valid true and
article_ready false. Its scope still has 10 open mission inputs, 11 open geometry
parameters, zero approved load cases and zero approved evidence packages.

No full-repository test pass or physical test is claimed. These results are not
EV-ARTICLE-005 qualification evidence. The resolver checks evidence declarations,
not the authenticity or adequacy of their referenced drawings.

## Next physical closure

Provide the fixture datum/pose and actual shoulder-output and dummy-elbow station
coordinates, then the two socket offsets and two insertion depths with tolerances
and drawing references. The resolver will calculate review lengths without
changing canon or locking a register. The complete shoulder drawing release
additionally needs mechanism selection, loads, stock sections, bearings, stops,
retention, BOM, analysis and coupon/article measurements.
