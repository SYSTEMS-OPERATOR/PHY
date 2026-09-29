# T56 A0-R1 validation record

Status: **FOCUSED VALIDATION PASS — INDEPENDENT REVIEW AND PHYSICAL EVIDENCE OPEN**

This record captures repository checks only. It is not inspection, test evidence,
a fabrication release or physical qualification.

| Check | Result |
| --- | --- |
| Focused A0, geometry-contract, resolver and canon regression suite | 62 passed; 0 failed; 0 skipped |
| Part STEP round trips | 19 passed |
| Neutral assembly STEP import | passed; 35 located instances |
| Repeat export determinism | 47 generated CAD/assembly/drawing files byte-identical |
| Discrete motion screen | 35 pose pairs; 0 registered centerline/AABB hits; not continuous proof |
| Convergence schema | valid |
| Fabrication release / physical evidence | false / false |
| Canon-controlled files | unchanged from PR base |

## Wider-suite environment result

`python -m unittest discover -s tests -p 'test_*.py'` ran 96 test entries.
Eighty-seven completed without a reported failure; nine modules could not import
because this runtime lacks optional/general-project dependencies (`pytest`,
`fastapi`, and `gymnasium`). These import errors are outside the A0-R1 change,
were not relabeled as passes, and do not replace CI.

## Engineering blockers preserved

- The conservative impact screen gives a 3.25 kN maximum bearing reaction
  against the currently unverified 3.0 kN per-bearing static threshold.
- The assumed 4 mm stop travel has no accepted compression curve or coupon.
- Supplier-dependent bearing, plunger, friction-stack, tether and bench details
  remain open.
- Clearance is discretely sampled; continuous swept-solid clearance, guarding
  and minimum tool access remain independent-review/dry-assembly work.
- All first-article and bench-test records remain blank and unapproved.
