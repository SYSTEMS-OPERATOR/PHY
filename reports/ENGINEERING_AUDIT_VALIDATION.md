# Engineering audit: scope and validation evidence

Date: 2026-10-12. Base: `c4792671c446033185de2953b8786e5ed508b876`
(PR #105 merge). Development branch: `agent/phy-engineering-audit`.

## Mission and selection

The 206-ID display milestone did not provide a combined account of canonical
record coverage, dimensions, dataset bindings, geometry and existing blockers.
Choose a shared adult identity inventory and an offline evidence audit before
adding unsupported records, measurements or mechanical assumptions. This makes
subsequent engineering improvements measurable while preserving the current
source gaps and qualification boundaries.

Inspection verified PRs #103–#105 were merged and no PRs were open. Current
default data have 206 display identities, 193 loaded records, 36 complete
declared dimension maps, 21 dataset bindings, 30 dimensional primitive estimates,
24 resolved declared anatomical-reference sets and no reference declarations.
Thirteen facial identities lack source records. All 193 loaded records lack
references, joint interfaces and mounts. Geometry remains unresolved in 163
records and measurements unknown in 157. Eight record-blocker categories remain.

## Implementation boundaries

- Move the existing adult inventory/static source lookup to
  `skeleton/anatomical_inventory.py`; preserve the old import locations and
  exact Studio coverage output.
- Add `skeleton/validation/engineering_audit.py` and
  `bin/audit_engineering.py`: independent global/regional observations and all
  206 identity rows with source hashes, canonical data, proxy correspondence,
  dataset binding hashes, declared provenance, reference resolution, blockers
  and evidence needs.
- Reuse strict JSON serialization and staging/rollback to publish JSON,
  Markdown and a hash manifest. Match the canonical export's input fingerprint.
- Add 13 regression tests, document the observation/qualification boundary and
  include audit checks in the existing Studio CI suite.

No bone module, source measurement, project profile, canonical schema, root
authority document, SOPHY canon, A0 article, mechanical transform, fabrication
export gate or release status is changed. Source records remain 193; no proxy
dimensions are adopted into canonical data. Generated audit reports are runtime
outputs rather than checked-in claims of qualification.

## Validation

| Check | Verified result |
| --- | --- |
| Unchanged main baseline, Python 3.12 | 245 passed, 5 skipped, 927 subtests passed |
| Final full local pytest suite, Python 3.12 | **258 passed, 5 skipped, 938 subtests passed** |
| Focused audit regression suite | **13 passed, 11 subtests passed** |
| Audit with standard-library Python (no pytest or CadQuery) | **13 unittest tests passed** |
| Viewer syntax and bundle build | Both npm checks passed; checked-in bundle unchanged |
| Actual Three.js GLB/STL export logic | 206 distinct IDs, role filtering and provisional/source-gap provenance checks passed |
| Full Studio export including A0/source kit | Five models, 743 viewable parts, 26 maquette cuts |
| Current Studio deterministic repeat | All **24 files byte-identical**; both ZIPs pass `testzip()` with 23 entries |
| Fresh main comparison | All **23 nonarchive files byte-identical**, including every model, offline HTML, source kit, report and maquette artifact |
| Audit deterministic repeat | All **three report files byte-identical** across separate output roots; both file hashes verified |
| Audit/export input agreement | Canonical-record snapshot hash equals the existing review export manifest's source-record hash |
| Existing fabrication validation CLI outside checkout | Expected exit 1, 193 records, eight blocker categories |
| Both existing export/assembly CLIs outside checkout | Default validated mode rejects; explicit review succeeds with readiness false |
| SOPHY canon export | Version 1.0.0, stature/span **1676.4 mm**, validation passes |
| Audit CLI outside checkout and subsequent invalid dataset | Success produces a report; failed input exits 1 and preserves the previous report bytes |
| Whitespace and Python compilation | Passed |

The 13 new tests exercise real counts and independent regional sums, preserved
Studio coverage, source/dataset hashes, missing records, null measurements,
partial/invalid dimensions, proxy changes without source adoption, dataset
binding gaps, malformed reference source types, side ambiguity, parent cycles,
malformed child declarations, unknown/duplicate IDs, nonfinite values, source
lookup failures, partial/empty snapshots, runtime/order isolation and CLI
failure preservation. Synthetic complete records remain anatomically
unqualified and fabrication release remains false.

Optional local checks: four PyBullet-dependent modules and one Docker check were
skipped. Browser smoke could not launch because the Playwright Chromium
executable is absent. The existing CI browser step remains enabled. Python 3.11
was not available locally; the Python package workflow covers that version.

The earlier cached and fresh main-worktree Studio ZIP comparisons could not be
accepted: those generated archives lacked a central directory. No baseline ZIP
byte-equality claim is made. The unchanged nonarchive contents and two valid,
byte-identical current ZIPs are the verified evidence; the audit does not alter
the Studio archive writer.

## Reproduce

```bash
python -m pytest -q
python -m unittest discover -s tests -p test_engineering_audit.py -q
python bin/audit_engineering.py --output-root /tmp/phy-audit-one
python bin/audit_engineering.py --output-root /tmp/phy-audit-two
diff -qr /tmp/phy-audit-one /tmp/phy-audit-two
npm run check --prefix studio
npm run build --prefix studio
python bin/export_phy_studio.py
npm run test:logic --prefix studio
python bin/export_phy_studio.py --output /tmp/phy-studio-repeat
diff -qr studio/dist /tmp/phy-studio-repeat
```

The full Studio export requires CadQuery 2.7.0. The audit CLI does not.
Missing optional simulation/browser tools are reported rather than treated as
successful checks.

## Limits and next work

This is an observation system, not a new physical qualification system. Complete
declared values, cited strings and resolved anatomical references are each
weaker than qualified measurements, verified evidence and working mechanics.
Report completion does not grant validated export or fabrication approval.

Next bounded source work: add the 13 facial canonical identities with explicit
unknown dimensions/geometry and supported identity references. Use the audit to
verify that only the appropriate coverage dimensions change. Follow with a
small-region anatomical relationship contract distinguishing sutures,
articulations, cartilage and soft-tissue suspension before asserting transforms
or joint behavior. No automatic merge or development-branch deletion is part of
this work.
