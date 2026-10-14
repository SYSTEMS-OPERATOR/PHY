# PHY-RECOVER-072 validation record

14 October 2026. Recovered PHY checkpoint d8afe81b75c09719cbfe7c8f001b1075590c493c;
main baseline 2064af478d6554f66900f655afdd64c8869a16b5. Source custody is independently
verified in PHY-REF PR 1. This report distinguishes software checks, tabular
transcription, anatomical qualification and physical release.

## Results

| Check | Verified result |
| --- | --- |
| Full local pytest suite | 295 passed, 5 skipped, 1,014 subtests passed; 63.18 s |
| Reference registration/correspondence after final CLI lineage change | 7 unittest methods passed |
| PHY-REF malformed-input/acquisition/GLB suite | 11 methods passed locally and in CI |
| Independent study transcription | Seven sources / 126 native mean+SD rows match; three rejected rows remain rejected |
| Main / continuation complete maps | 36 / 40 of 206; four additions; target shortfall 32 |
| Newly source-backed fields | 128 (93 preserved + 35 thoracic widths) across 32 candidates |
| Improved partial maps | 28; original generic whole-vertebra unknowns retained |
| Identity / nonregression | 206 retained; all 36 prior complete maps and all 93 recovered fields unchanged |
| NIH source-label crosswalk | 36 canonical IDs / 46 source mesh nodes; lumbar-6 held unmapped |
| Canon | Version 1.0.0, height and span 1676.4 mm; validation passed |
| Reports / review export | 11 audit/report files and exchange payloads repeated byte for byte |
| Require-target gate | Expected exit 2; no false declaration of 72 maps |
| Validated fabrication export | Expected exit 1; no release; previous review payloads preserved |
| Geometry / fabrication additions qualified | 0 / 0; full record validation remains 0/206 |
| A0 / authority / geometry profiles | No changes to BODY/MIND/SOUL, canonical geometry, project profiles or A0 mechanism |
| Whitespace/diff check | Passed |

Five local skips comprise four optional PyBullet-dependent modules and the Docker
availability test. CadQuery geometry/export tests ran. Initial full-suite
collection failed for missing FastAPI and Gymnasium, then for the newer Starlette
test client's missing httpx2 dependency. Installing the already-declared runtime
dependencies resolved collection; repository dependency specifications were not
changed. The complete suite then passed. Existing warnings for missing legacy
dataset bindings remain visible and are not interpreted as admitted measurements.

No new browser behavior or authored Studio proxy geometry is introduced. Recovered
checkpoint CI passed Python and Studio workflows, including checks/build,
deterministic GLB/STL exports and browser smoke:
[Python run](https://github.com/SYSTEMS-OPERATOR/PHY/actions/runs/38501691251),
[Studio run](https://github.com/SYSTEMS-OPERATOR/PHY/actions/runs/38501691250).
The continuation adds the new reference tests to Studio CI; final branch checks
are visible on [PR 110](https://github.com/SYSTEMS-OPERATOR/PHY/pull/110).
Node/browser tools were not installed locally for a second browser run.

## Exact local commands

The machine record reports each command, working directory, exit code, duration
and output tail in reports/recovery/local_checks.json. Commands below ran from
the recovered PHY checkout. The repeated root is an independent scratch output,
never a second canonical source.

- `python -m pytest -q` — exit 0 (expected 0).
- `python bin/audit_engineering.py --dataset female_21_baseline --output-root /workspace/scratch/0559b9211f09/verification/recovery-repeat` — exit 0 (expected 0).
- `python bin/audit_dimension_expansion.py --output-root /workspace/scratch/0559b9211f09/verification/recovery-repeat` — exit 0 (expected 0).
- `python bin/audit_dimension_expansion.py --output-root /workspace/scratch/0559b9211f09/verification/recovery-repeat --require-target` — exit 2 (expected 2).
- `python bin/audit_craniofacial.py --output-root /workspace/scratch/0559b9211f09/verification/recovery-repeat` — exit 0 (expected 0).
- `python bin/audit_reference_correspondence.py --reference-root /workspace/scratch/0559b9211f09/PHY-REF --output-root /workspace/scratch/0559b9211f09/verification/recovery-repeat` — exit 0 (expected 0).
- `python bin/export_fabrication.py --mode review --output-root /workspace/scratch/0559b9211f09/verification/recovery-repeat` — exit 0 (expected 0).
- `python bin/export_fabrication.py --mode validated --output-root /workspace/scratch/0559b9211f09/verification/recovery-repeat` — exit 1 (expected 1).
- `python bin/export_sophy_canon.py --height-mm 1676.4 --output /workspace/scratch/0559b9211f09/verification/recovery-checks/canon.json --validation-report /workspace/scratch/0559b9211f09/verification/recovery-checks/canon-validation.json` — exit 0 (expected 0).
- `git diff --check` — exit 0 (expected 0).

Additional checks:

- `python -m unittest tests.test_reference_registration tests.test_dimension_evidence -q` — 17 passed before the final protocol admission check.
- `python -m pytest tests/test_reference_registration.py tests/test_dimension_evidence.py tests/test_facial_source_records.py -q` — 26 passed, 58 subtests passed at that intermediate checkpoint.
- `python -m unittest tests.test_reference_registration -q` — seven passed after final units/input-packet-hash lineage changes.
- `python bin/audit_reference_correspondence.py --reference-root ../PHY-REF` — 36 IDs checked; repeated JSON/manifest hashes equal.
- `python verification/reverify_sources.py` from the shared workspace — independently extracted selected female columns and compared 126 native means/SDs exactly; results in reports/recovery/source_reverification.json.
- PHY-REF: `python -m unittest discover -s tests -v`; `python tools/acquire_nih.py --all`; `python tools/audit_assets.py --output ../verification/recovery-ref-audits` — 11 passed, all files reused, cached/repeated audits equal.
- Fresh anonymous PHY-REF clone at 3576c8a8ae5766ab684a221813de13e0578003be followed by `git lfs pull` — all three original sizes and SHA-256 hashes match.

Original cached mesh audits and reduced GLBs were not regenerated. The native
preview projections were visually inspected. Their geometry remains reference
only; software parsing and visual agreement do not validate anatomy, tissue
mechanics, joints or fabrication.

## Remaining engineering blockers

72/206 complete maps are not defensibly available from the qualified evidence.
NIH native scale/segmentation error, actual anatomical landmark packets,
canonical registration, whole-vertebra extents, hindfoot contact surfaces,
materials/load acceptance, mounts and joint interfaces remain unresolved. The
existing A0 bearing/stop release issues are independent. No automatic merge,
branch deletion, new repository, NASA compliance or physical readiness is claimed.
