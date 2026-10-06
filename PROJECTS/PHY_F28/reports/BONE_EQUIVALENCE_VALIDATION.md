# Bone-equivalence core / validation checkpoint

Reviewed 2026-10-06 UTC against PHY main
`ae81de1cd3ac555a2ec57e540f9882a3a550adfe` (PR #97).

Status: **bounded distribution review; not anatomical dimensional adoption or
fabrication release**. The proxy layout is project-local provisional geometry.
Physical evidence is unmeasured. Canon 1.0.0 and the A0-R1 packet are unchanged.

## What changed

All 206 adult identities have a separate ledger disposition. There are **71
individual core bone proxies**, 128 grouped skull/hand/foot identities and seven
unrepresented ear/hyoid identities. The 119 reference meshes split into 71 bone
proxies, 29 grouped/form meshes, 12 hardware envelopes, five supports and two
envelopes. 193 source BoneSpec records remain unchanged; 13 facial records are
still missing. None of these counts establish dimensional or functional fidelity.

The prior aggregated spine, thorax, forearm, scapular and pelvic **visual shapes**
are replaced, not the canonical bone modules. The source kit and A0 geometry are
not removed/rebuilt. The earlier passive 26-cut maquette has its own preserved
schedule and is byte-identical to the base packet, not a skeletal equivalent.

Bone IDs, roles, grouping, input pointers, a layout hash and unverified/unmeasured
status accompany the meshes/coverage JSON. The offline Studio adds coverage,
bone-ID search and bone-only view/GLB/STL export. Export selection uses the same
tested role logic as the viewer, rather than material color as bone classification.

## Executed checks

| Check | Result / limits |
| --- | --- |
| Focused Python engineering suite below | 102 tests and 381 subtests passed; no skips in this suite |
| Inventory/distribution regressions | Exact 206 identities and regional counts; exact 71 core identities; correct grouping/exclusions; missing 13 source records exposed |
| Mutation tests | Reject duplicate/unknown mappings, wrong regions, non-bone IDs, group overlap, missing provenance and unearned physical evidence; a lost patella is visible even with unchanged mesh count |
| Placement/topology | Ordered C1–C7/T1–T12/L1–L5/sacrum/coccyx; 24 separate open ribs at corresponding T levels; true/false/floating relationship labels; separate radius/ulna, anterior patellae and plate proxies |
| Mirror and meshes | Exact right-derived left vertices, reversed triangle winding and mirrored metadata; all reference meshes closed, nondegenerate and positive-volume |
| Canon CLI | Canon 1.0.0, H = span = 1676.4 mm; pass true, zero issues |
| A0/component regression | Existing A0 located bounds, scope/contracts/calculations and source-component tests pass; A0/kit files are unchanged |
| Node syntax/build | App, shared part-role helpers and both test scripts parse; offline bundle rebuild succeeds |
| Actual three.js exports without WebGL | 71 unique bone IDs in GLB mesh extras, filtering/search/hardware exclusion, full-frame STL at explored 1700 mm height; no visual rendering claimed |
| Repeat export | Two full five-model exports match recursively, including HTML, JSON, STL, reports, manifests and ZIP bytes; ZIP CRC checks report no bad member |
| Standard-library package regression | Two three-model exports match byte-for-byte; ZIP opens and carries the complete 206-row ledger/layout report |
| Preserved review files | Checked-in HTML/three bone reports match regeneration; all maquette files match the base packet |
| Protected files | No diff in BoneSpec modules/schema/base, BODY/MIND/SOUL, SOPHY canon or PROJECTS/T56_CARBON |

The initial local archive comparison encountered an incomplete default-output ZIP;
a completed regeneration and fresh repeat produced matching, CRC-valid archives.
The repeatable-package regression now checks archive completeness as well as bytes.

## Not executed / not passed

- `npm run test:browser --prefix studio` could not launch: the local Playwright
  executable is absent. Both the pinned browser and a stable-version download
  failed (non-ZIP responses / gateway failure). **Local visual WebGL, interaction,
  screenshot and mobile-layout checks are unrun**, not passed. The extended CI
  smoke checks cover bone-ID selection, 71-bone visibility and GLB metadata; they
  remain required before merge.
- `python -m pytest -q` stopped during collection: unchanged privacy/cognition/
  social/distributed modules require missing `fastapi` and `gymnasium`. Four
  collection errors and three dependency skips; **not a full-repository pass**.
  No unrelated dependencies/modules were changed to conceal these results.
- No STEP conversion/round-trip for these polygonal review overlays. The existing
  A0 B-rep is not changed or exported anew as a fabrication release.
- No continuous motion/clearance proof, anatomical surface fidelity, joint-center
  adoption, strength qualification, fabrication inspection or physical testing.

## Reproduce

```bash
npm ci --prefix studio
npm run check --prefix studio
npm run build --prefix studio
python bin/export_phy_studio.py --sync-review
npm run test:logic --prefix studio
python -m pytest tests/test_bone_equivalence.py tests/test_phy_studio.py \
  tests/test_sophy_geometry_canon.py tests/test_sophy_project_canon_bindings.py \
  tests/test_reference_dimensions.py tests/test_shoulder_dimensions.py \
  tests/test_t56_a0_geometry.py tests/test_t56_a0_scope.py \
  tests/test_t56_a0_validation.py tests/test_t56_geometry_contracts.py \
  tests/test_bone_count_pytest.py tests/test_component_3d.py -q
python bin/export_phy_studio.py --output /tmp/phy-bone-repeat-review
diff -qr studio/dist /tmp/phy-bone-repeat-review
cmp studio/PHY-Studio.html studio/dist/PHY-Studio.html
diff -qr PROJECTS/PHY_F28/maquette studio/dist/maquette
PYTHONPATH=. python bin/export_sophy_canon.py --height-mm 1676.4 \
  --output /tmp/phy-bone-review-canon.json \
  --validation-report /tmp/phy-bone-review-canon-validation.json
# Requires an installed browser; do not treat failure to launch as a passed smoke.
npm run test:browser --prefix studio
```

## Independent-review checklist / next bounded work

1. Run the browser smoke in a browser-capable environment. Inspect front, side,
   back and T/A poses, role filters and every region's coverage; do not accept
   mesh counts alone as anatomical equivalence.
2. Keep each unknown dimension/articular interface visibly unverified. In
   particular the vertebral bodies, scapular plates and adult hip plates are
   topological placeholders, not replicated bone morphology; cartilage and joints
   connecting ribs to sternum/costal margin are not modeled.
3. Next PR: one right hand with 27 individual bone identities (eight carpals,
   five metacarpals, 14 phalanges), explicit opposed-thumb/radial-side topology,
   then mirror left. Follow with feet and craniofacial/ear/hyoid distribution.
   Do not mistake the current generic digit fan for an anatomically correct thumb.
4. Separately resolve measured osteometric endpoints/morphology and functional
   joints before any near-1:1 **dimensional** equivalence or fabrication claim.
   NASA/ANSUR ergonomic envelopes do not silently supply bone lengths.
5. Preserve deterministic Vitruvian-derived canon: scale, materials and mechanisms
   may change; canon does not drift. A0's 317 mm dummy station remains project-local.

Commits are separated into inventory/audit, corrected core geometry/inspection,
and validation/package checkpoints. No auto-merge or fabrication approval.
