# PHY Studio: native NIH comparison and pelvic draft review

Baseline PHY main: `675cd8585e14a1a0662fb4798346ce383c4e5401`.
Reference baseline: `eaf86d4237b57a0e096c514f74fbca5a9da697df`.
Consumed immutable PHY-REF review revision:
`0cafc8e7f387d009a666b3b3b44bfae29904c53c` /
[PR 2](https://github.com/SYSTEMS-OPERATOR/PHY-REF/pull/2).
That packet is proposed review evidence; accepted coordinates remain null.

## Delivered scope and preserved authority

| Item | Before | Proposed result |
| --- | --- | --- |
| Studio models | Five | Same five plus NIH pelvis and skin native previews |
| NIH display geometry | PHY-REF only | Exact derived bytes embedded in offline Studio; originals stay PHY-REF |
| Native frame | Source world Y-up / meters | Preserved; only 1000 mm/m display conversion |
| Anatomical/PHY registration | Unknown | Unknown; separate views, no automatic overlay or fit |
| Pelvic feature review | Open | Five definitions and unaccepted preview capture with triangle lineage |
| Canonical identities / maps | 206 / 40 complete declared maps | Unchanged; no preview point becomes a dimension |
| SOPHY 1.0.0 height/span | 1676.4 / 1676.4 mm | Unchanged |
| A0 article | 35 located instances; bearing/stop evidence blockers | Preserved; no redesign |
| Physical readiness | Unqualified | Unqualified |

`studio/reference_lock.json` pins all consumed metadata, inspector, rights and
preview files. The exporter uses an explicit local checkout, verifies sizes and
hashes before publication, reuses PHY-REF's static GLB inspector and reads no
original GLB. The two preview files total 824,908 bytes. CI uses an immutable
checkout with LFS disabled; no source acquisition occurs in PHY's Studio build.

Native views disable height, pose, palette, explosion and canonical proxy filters.
Camera views use source-axis labels, and no floor is inferred from native Y=0.
Native GLB download preserves exact pinned bytes. JSON separates source evidence
and draft points; native STL/reinterpreted mesh export is unavailable. The
existing armature exporters, NASA evidence and maquette remain available.

Draft candidates retain mesh labels, original source node, preview node/face,
vertex indices, barycentric weights, native meters and packet/source/preview
hashes. Original-face correspondence cannot be inferred through decimation.
It remains null, as do accepted landmarks and the canonical transform.

## Local verification

```bash
PHY_REF_ROOT=/path/to/pinned-PHY-REF python -m pytest -q
PHY_REF_ROOT=/path/to/pinned-PHY-REF python -m pytest tests/test_studio_references.py tests/test_studio_evidence.py tests/test_reference_registration.py tests/test_phy_studio.py -q
npm run check --prefix studio
npm run build --prefix studio
python bin/export_phy_studio.py --reference-root /path/to/pinned-PHY-REF --sync-review
npm run test:logic --prefix studio
python bin/export_phy_studio.py --reference-root /path/to/pinned-PHY-REF --output /tmp/phy-native-repeat
diff -qr studio/dist /tmp/phy-native-repeat
python -S bin/export_phy_studio.py --without-a0 --reference-root /path/to/pinned-PHY-REF --output /tmp/phy-native-stdlib
python -S bin/export_phy_studio.py --without-a0 --output /tmp/phy-native-opt-out
git diff --check
```

The complete local suite passed **309 tests, 5 optional skips, 1,151 subtests**,
with 328 existing missing-metric warnings. The focused suite passed 29 tests and
392 subtests. The skips are optional PyBullet/Docker paths. Source/GLB logic
checks passed, including modified checksums/lengths, missing files, symlink escape,
native unit conversion, no original reads, and preview triangle reconstruction.
Existing export checks retained all 206 IDs and 1700 mm exploratory STL height.

Nineteen pre-existing geometry/layout/maquette payloads match the preceding
Studio build byte for byte: six model JSON/STL files, seven bone/layout reports
and six maquette files. The seven-model package has 32 distribution files,
including a 31-entry CRC-checked ZIP. A repeat build matched all files. Fresh
standard-library builds with and without references succeed.

PHY-REF's 17 tests passed locally; its
[remote integrity/geometry run](https://github.com/SYSTEMS-OPERATOR/PHY-REF/actions/runs/38649892743)
also passed, including original acquisition/hash/audit checks and the new pending
packet check. This run does not anatomically qualify the assets.

## Browser verification gate

`npm run test:browser --prefix studio` checks existing exports, all seven models,
native bounds/scale, exact GLB downloads, asynchronous switching, preview capture
and draft JSON, preserved canon, offline network behavior and 390 px layout.
The captured test point checks a cortical triangle; it does not identify ASIS.
Exported drafts also require the independent PHY-REF geometric validator.

The 2026-10-19 repair reproduced the Actions failure from
[run 38651649966](https://github.com/SYSTEMS-OPERATOR/PHY/actions/runs/38651649966):
the first full-page screenshot enlarged the in-flow WebGL canvas and its stage
to 445,560 px. The next screenshot attempted a 1440 by 445,811 bitmap and crashed
Chromium. The canvas is now positioned inside its stage without contributing
intrinsic dimensions to the flex/grid layout. A regression checks bounded,
stable stage dimensions across 1800, 1024, 1440 and 390 px viewports and after
full-page capture; it fails against the original layout before the browser crashes.

Visual review also exposed stale parent transforms during camera framing after
switching into and out of the native views' 1000 mm/m display scale. Refreshing
world matrices before measuring visible meshes restores the pelvis framing and
the armature on return, including phones. Browser checks now verify projected
part centers remain on-screen and the model remains large enough to inspect.
The asynchronous-switch check distinguishes all 230 source parts from the 228
default-visible parts, then verifies that enabling the two envelope meshes shows
all 230. No source geometry, native coordinates or layer defaults were changed.

The complete seven-model browser smoke now passes locally with no browser errors
or HTTP requests. Playwright's ordinary browser download still returns invalid
ZIP data here, so the local run used `@sparticuz/chromium@153.0.0` with a
single-process SwiftShader launch wrapper. The repository's ordinary Actions
launch remains unchanged. Desktop, pelvis, exterior, phone and evidence-dialog
screenshots were visually inspected. The exported draft also passes PHY-REF's
independent geometric validator as `preview_geometry_verified_only`.

The Studio workflow's Python checks passed **215 tests and 937 subtests** during
the repair; syntax/build, native/bone export logic and deterministic rebuild
checks also pass. Generated bundle and offline review HTML are synchronized.
The normal Actions browser check must also pass before this PR is marked ready;
see [PR 112 checks](https://github.com/SYSTEMS-OPERATOR/PHY/pull/112/checks) for its
current result.

## Remaining engineering decisions

Review PHY-REF PR 2 and this Studio slice before merge. Then appoint two
independent original-surface reviewers for the sacrum and fused left hip.
Establish source metric calibration, segmentation limitations and point/patch
definitions, followed by SYSOP-approved repeatability, uncertainty and registration
budgets. Accept original points only in a separate reviewed packet. The source
lumbar-6 context remains unresolved for the spinal interface.

NASA HIDH/OCHMO-HB-004 and prior references remain positioning, measurement-frame
and operating-condition comparisons. They do not replace osteometry, define
mechanical limits, rescale SOPHY in microgravity or establish NASA compliance.
No tissue simulation, new repository, major architecture or automatic merge is
part of this change. Software tests do not prove fabrication or physical readiness.
