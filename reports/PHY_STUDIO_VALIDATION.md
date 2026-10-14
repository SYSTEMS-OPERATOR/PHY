# PHY Studio evidence refresh — 2026-10-14

Status: **OFFLINE ENGINEERING REVIEW AND REPRODUCIBLE FORM-STUDY PACKAGE**.
Full-scale structural fabrication release: **false**. Physical evidence:
**unmeasured**. Reviewed engineering baseline:
`c3e260cddf022f9acec1dfa457efeeb50f4750fc` (PHY main, 2026-10-14T09:01:06Z).

## Delivered behavior

- All 206 source records and individual display identities remain available.
- Engineering reports 40 complete declared maps, 28 improved partial maps and
  128 traced additions from PHY-DIM-072. The 72-map target is short by 32.
- Component inspection separates display envelopes from unscaled canonical
  declarations. Traced fields expose native mean/SD, study/cohort/side, landmark
  method, source revision/hash, derivation, rights and uncertainty. Earlier
  declarations without field protocols remain identified as declarations.
- NASA presents 20 unsuited, 16 suited unpressurized and 16 suited pressurized
  motion pairs, 11 external anthropometric comparisons and all 17 source issues.
  Printed signs and conditions are retained; source selection does not move the
  model or set mechanical constraints.
- Explicit baseline pins and fresh existing audits reject stale canonical,
  proxy or catalog inputs. No cached audit report supplies Studio counts.
- The offline HTML, review JSON and package manifest carry evidence/build hashes.
  The engineering revision describes the reviewed data baseline; content hashes
  identify this viewer build. No wall clock or Git HEAD enters deterministic data.
- The ZIP is closed and CRC checked in a staging directory before atomic replacement.
  A failed CRC check retains the prior archive; staging files are cleaned up.

## Verification

| Check | Result |
| --- | --- |
| Full local Python suite | 305 passed, 5 skipped, 1,147 subtests passed; 328 warnings; 72.16 s |
| Optional skips | Four PyBullet-dependent modules and one Docker-dependent test |
| New evidence and archive regressions | 10 unittest methods passed; 128 field-level subtests in pytest |
| Focused engineering/Studio suite before the archive addition | 51 passed, 410 subtests passed |
| Viewer syntax/build | Passed |
| Actual GLB/STL logic | Passed; 206 unique bone IDs, role/provenance extras, 1700 mm explored STL height |
| Browser | Headless Chromium 153 with real WebGL/SwiftShader; zero page/console errors |
| New browser coverage | 40/206 counts, partial/unknown records, landmark/SD detail, NASA conditions/conflict, unchanged pose, evidence in review download |
| Existing browser coverage | Five models, selection/ruler, height/pose/isolation, 228-mesh complete-frame GLB and 206 bone-only meshes, maquette ZIP |
| Offline behavior | No HTTP(S) requests while inspecting evidence and NASA references |
| Mobile | 390 px; controls reachable, tables contained, numerical cells readable, close control retained during scroll |
| Minimal build | `python -S` with `--without-a0` passes; three body models and maquette, standard library only |
| Determinism | Fresh complete output trees, HTML, manifests and ZIP match byte for byte |
| Nonregression | 21 existing geometry, source, layout and maquette payloads unchanged byte for byte |
| ZIP inspection | 25 entries; CRCs and embedded files independently checked |
| Canon | Version 1.0.0, height/span 1676.4 mm; all 206 identities retained |

Exact commands, from the PHY repository root:

```bash
npm ci --prefix studio
npm run check --prefix studio
npm run build --prefix studio
python bin/export_phy_studio.py --sync-review
npm run test:logic --prefix studio
python -m pytest -q
python -m unittest tests.test_studio_evidence tests.test_phy_studio.PHYStudioTest.test_package_archive_replaces_only_after_crc_validation -q
python -S bin/export_phy_studio.py --without-a0 --output /tmp/phy-studio-stdlib
python bin/export_phy_studio.py --output /tmp/phy-studio-repeat
diff -qr studio/dist /tmp/phy-studio-repeat
cmp studio/PHY-Studio.html studio/dist/PHY-Studio.html
npm run test:browser --prefix studio
git diff --check
```

Local repeat/standard-library directories were under the workspace `verification/`
folder. Playwright's browser download returned invalid ZIP data locally. QA used
`@sparticuz/chromium@153.0.0` from npm, extracted without archive ownership changes,
with the existing `PHY_CHROMIUM_EXECUTABLE` override. No browser dependency was added
to PHY. A custom font configuration and an ambiguous test selector were corrected
before the final browser pass. CI uses its existing Playwright browser installation.

Local inspection also found incomplete ZIP copies despite successful exporter
exit codes. The new archive publication check and final CRC/content comparisons
verify the actual artifact; success is not inferred from a command log.

See `reports/studio_evidence/verification.json` for content fingerprints and exact
nonregression paths, and the adjacent desktop/mobile screenshots for visual QA.

## Engineering limits

Declared completeness is not dimensional fidelity or physical qualification.
T6 superior body width remains withheld; whole-vertebra envelopes, joint axes,
articular surfaces and interfaces remain unresolved. No NASA reference becomes
an armature stop, joint center, microgravity scaling rule or compliance claim.
ANSUR II female age-28 means remain separate from NASA populations and donor geometry.
Cohort SD is not a manufacturing tolerance or a measurement-error bound.

NIH mesh loading/registration is a subsequent reviewed slice; originals remain in
PHY-REF. A0 retains its exact 19-part / 35-instance geometry, 3.25 kN bearing screen
against the unverified 3.0 kN threshold, stop-response and bench-evidence blockers.
The passive 26-cut maquette retains its independent, unmeasured physical status.
