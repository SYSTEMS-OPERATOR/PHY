# PHY-DIM-072 validation record

13 October 2026. Base PHY main:
`2064af478d6554f66900f655afdd64c8869a16b5`.
PHY-REF inspected at `5d4cd1e841ec275580e6634fee760e9615f02c30`.

**Result: 40 complete declared dimension maps, 28 improved partial maps,
138 unknown maps; exactly 206 identities. The target of 72 is not met.**
Four complete additions and 93 new source-backed fields are retained. Every
original complete dimension map retains its values. All new generic and named
spans resolve to reviewed native table rows with explicit units and side/cohort
semantics. No source field or unknown was removed to inflate completeness.

## Software verification

| Check | Result |
| --- | --- |
| Python engineering/export/Studio/canon workflow suite, including new evidence tests | 193 tests passed; 783 subtests passed |
| Focused evidence + engineering audit run before integration | 22 tests passed; 39 subtests passed |
| `npm run check --prefix studio` | Passed |
| `npm run build --prefix studio` | Passed |
| Studio mesh export | Five models, 743 viewable parts, 26 maquette cuts |
| `npm run test:logic --prefix studio` | Passed: 206 IDs, bone filtering/search, actual GLB/STL exports and 1700 mm exported display height |
| SOPHY canon export/validation at 1676.4 mm | Canon 1.0.0, stature/span equal; canon checks passed |
| Canonical review export | Published review package; validated_ready false; fabrication_released false |
| Canonical review package repeated to an independent root | Every manifest and referenced payload byte-for-byte equal |
| Engineering and dimension-expansion audits repeated to independent root | All six report/manifest files byte-for-byte equal |
| `audit_dimension_expansion.py --require-target` | Expected exit 2; target_met false; shortfall 32 |
| Identity checks | No duplicate IDs, invalid IDs or parent cycles; unresolved legacy relationship declarations remain explicit |
| Authority/canon/project profiles/A0 diff | No changes |
| `git diff --check` | Passed |

The selected Python command is the existing Studio workflow's engineering suite
plus `tests/test_dimension_evidence.py`. It covers source facts and absent
provenance, invalid numerical data, explicit side handling, unknown unit/frame
changes, canonical/export behavior, bone inventory, unchanged display geometry,
craniofacial review and A0 geometry. It is not a claim that every unrelated
legacy runtime, cloud or hardware test was run locally.

Initial integration failures exposed stale checked-in canonical/craniofacial
fingerprints and assertions expecting the historical 36-map/13-citation state.
The review exports and dependent audit packages were regenerated. Snapshot
assertions were updated to the reviewed current state. Foot-source checks still
require explicit admitted values or unknowns, and the new baseline comparison
checks all 206 maps and every original complete map. The full selected suite
then passed.

## Browser verification limitation

The local Playwright browser smoke test could not launch because its Chromium
executable was absent. The prescribed headless Chromium installation was
attempted; the browser download returned an unusable/truncated archive and
installation failed. Therefore local visual/browser interaction remains
unverified. No viewer UI or authored proxy geometry was changed. Existing
browser CI remains enabled; no browser assertion was removed or skipped in the
repository workflow.

## Engineering status

Records passing the current full engineering/fabrication record validator:
**0/206**, reported separately from the 40 complete dimension maps. All 206
records still lack physical joint interfaces and mounts. No material allowable,
contact geometry, joint center, registered surface, bench evidence or NASA/flight
qualification is supplied by this slice. Newly changed records retain explicitly
unknown mass, and the validated exporter remains blocked.

Two reported native entries remain rejected: C2 HT until its odontoid/span
definition is confirmed, and the internally inconsistent T11 posterior height.
NIH meshes remain unadmitted for metric use; NASA sources inform measurement and
interface methodology without supplying invented bone measurements.

The source review and next fabrication tasks are in `docs/PHY-DIM-072.md`.
The complete 36-candidate evidence matrix, all changed values and all 206
before/after maps are in the dimension-expansion reports. This is a reviewable
evidence-limited delivery, not a declaration that PHY-DIM-072 has reached 72.

## Recovery continuation — 14 October 2026

The recovered 93 fields were preserved; 35 thoracic body widths were added.
Current source-backed field count is 128; complete maps remain 40/206.
NIH originals are now preserved and independently retrievable in PHY-REF.
Fresh commands, results, source checks and limits are in RECOVERY_VALIDATION.md.
