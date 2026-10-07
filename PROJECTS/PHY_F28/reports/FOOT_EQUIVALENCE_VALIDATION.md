# Individual foot bone-equivalence / validation checkpoint

Reviewed 2026-10-07 UTC against PHY main
`214c503ff595576863b545a3e46891dd515f98ed` (PR #99).

Status: **bounded identity/distribution review, not dimensional equivalence or
fabrication release**. Every new section, station, elevation and gap is
project-local provisional display geometry. Physical evidence is unmeasured.
SOPHY canon 1.0.0, core/hand layouts and the separate A0-R1 article are unchanged.

## Resulting coverage

| Disposition | Before | This checkpoint |
| --- | ---: | ---: |
| Individually identified project proxies | 125 | 177 |
| Grouped skull/foot identities | 74 | 22 (skull) |
| Unrepresented ear/hyoid identities | 7 | 7 |
| Individually represented foot bones | 0 | 52 / 52 |
| Expected adult identities | 206 | 206 |
| Missing facial source records | 13 | 13 |

Each foot has seven separate tarsals, five metatarsals and 14 toe phalanges,
including two in the hallux and three per other toe. Talus above heel, anterior
navicular/cuboid, medial/intermediate/lateral cuneiforms, medial hallux and five
toe chains are inspectable. Selected parent labels are partial topology, not
a complete joint/contact graph. Ellipsoids and tubes do not reproduce bones.

The old grouped foot ellipsoids are replaced. The 149 non-foot meshes in the
refined reference, existing BoneSpec records and the passive 26-cut maquette
are preserved. One foot profile supplies sections, stations, heel offset and
gaps. Both feet expose its hash/input pointers. Left vertices, winding,
stations, anchors, chain parents and ankle provenance reflect the authored right.

## Executed checks

| Check | Result / limits |
| --- | --- |
| Python 3.12.14 focused engineering suite | 117 tests and 637 subtests passed; no skips |
| Python 3.11.16 foot/hand/bone/Studio/canon/bindings suites | 50 tests and 400 subtests passed; one optional CadQuery viewer test skipped in the isolated environment |
| Identity/source | Independent 26-ID lists per side; all 52 existing foot source records retain wholly unknown dimensions; lost hallux identity remains visible despite unchanged mesh count |
| Frame/distribution | Both sides at 0°, 45° and 80° using cohort/1676.4/1800 mm overlays; tarsal order, medial hallux, ordered toe chains, second metatarsal longest and first section thickest in this display |
| Datum/gaps | Calcaneus mesh reaches floor and heel datum; all foot vertices remain above floor and within lateral external breadth; second-toe nominal terminal station closes the external length; trimmed endpoints and chain stations are consistent |
| Parameters/evidence | 20 invalid-profile mutations rejected, including missing/extra IDs, bad vectors/numbers, chain counts/roots/order, reversed tarsals, displaced heel, negative sections, invalid/consuming gaps and altered metatarsal order; uniform scaling, pose invariance, layout hashes and unverified/unmeasured flags passed |
| Meshes/reflection | All 201 reference meshes are closed, nondegenerate and positive-volume; foot reflection includes winding, stations, centers, anchors, side-qualified topology and provenance |
| Node syntax/build | Checks and offline bundle rebuild passed |
| Actual Three.js exports without WebGL | 177 unique bone IDs in GLB, including 52 foot identities with input pointers/layout hash/evidence; mirrored chain parents, hallux/heel search, role exclusions and complete-frame 1700 mm STL passed |
| Full five-model repeat | All 22 package files match byte-for-byte, including HTML, JSON, STL, reports, manifests and ZIP |
| Cross-version package | All 22 standard-library package files match byte-for-byte on Python 3.11.16 and 3.12.14; four generated ZIP CRC checks pass |
| Checked review files | All 12 checked-in HTML/report/maquette files match regeneration; all six maquette files and the 26-cut schedule match the base |
| Protected geometry/data | All 149 non-foot refined meshes and CORE_V1/HAND_V1 layout values match the base; no diff in BODY/MIND/SOUL, BoneSpec/schema/base, SOPHY canon, PROJECTS/T56_CARBON or maquette files |
| Canon CLI | Canon 1.0.0, H = span = 1676.4 mm; pass true, zero issues |
| Internal visual inspection | Actual right-foot mesh dorsal/medial projections inspected; this is not WebGL, articular-surface, contact or continuous-clearance validation |

An initial local repeat comparison found a truncated archive in the existing
`studio/dist` output. The packet was regenerated; subsequent complete-package
byte comparisons and ZIP CRC checks pass. No geometry adjustment was needed.

## Convergence status

Foot identity coverage is complete for this bounded checkpoint; 26 IDs per side
have separate inspectable meshes and source correspondence. Overall individual
identity coverage is **177 / 206**. The remaining 29 identities are 22 grouped
skull bones and seven unrepresented ear/hyoid bones. Thirteen of the grouped
facial identities still lack canonical source records.

Dimensional fidelity verified, physical evidence complete and fabrication
released all remain **false**. Sections and layout fractions are proposals;
external NASA/ANSUR envelopes do not supply individual bone measurements.
Nominal toe station closure is not exact mesh-envelope conformity or anatomical
endpoint adoption. Display gaps and parent labels are not qualified joint fits.

## Reproduce

```bash
npm run check --prefix studio
npm run build --prefix studio
python bin/export_phy_studio.py --sync-review
npm run test:logic --prefix studio
python -m pytest tests/test_foot_bone_geometry.py tests/test_hand_bone_geometry.py \
  tests/test_bone_equivalence.py tests/test_phy_studio.py \
  tests/test_sophy_geometry_canon.py tests/test_sophy_project_canon_bindings.py \
  tests/test_reference_dimensions.py tests/test_shoulder_dimensions.py \
  tests/test_t56_a0_geometry.py tests/test_t56_a0_scope.py \
  tests/test_t56_a0_validation.py tests/test_t56_geometry_contracts.py \
  tests/test_bone_count_pytest.py tests/test_component_3d.py -q
python bin/export_phy_studio.py --output /tmp/phy-foot-repeat
diff -qr studio/dist /tmp/phy-foot-repeat
python bin/export_phy_studio.py --without-a0 --output /tmp/phy-foot-stdlib
PYTHONPATH=. python bin/export_sophy_canon.py --height-mm 1676.4 \
  --output /tmp/phy-foot-canon.json \
  --validation-report /tmp/phy-foot-canon-validation.json
# Absent Chromium is not a browser pass.
npm run test:browser --prefix studio
```

Run the standard-library export and foot/hand/bone/Studio/canon/bindings suites
with Python 3.11 and 3.12, then compare the export directories recursively.

## Unrun / unresolved

- Local browser smoke failed to launch: the pinned Chromium headless executable
  is absent. No new local WebGL, surface-selection, visual/mobile or interaction
  pass is claimed. Extended remote Studio CI checks 177-bone coverage, hallux
  search and foot GLB metadata before merge. Do not wait for its long run to open
  the PR or auto-merge it.
- No local full-repository pass claimed; the focused suite does not cover every
  runtime subsystem. Python-package CI remains responsible for that scope.
- Measured foot endpoints, individual bone morphology, articular surfaces,
  sesamoids/variant anatomy, ankle/subtalar/toe axes, arches, tendons/ligaments,
  gait and working connections remain unresolved. The inventory is the existing
  adult 206 convention, not a claim that every adult has exactly this anatomy.
- No interference/motion proof, structural analysis, machining STEP qualification,
  fabrication or physical testing is asserted for these display meshes.

## Independent review and next bounded work

1. Confirm latest-head CI passes. Search `BONE_T_PHAL_1_2_R` and
   `BONE_CALCANEUS_L` in Studio; inspect both feet from top and side. Confirm
   bone-only exports retain all 52 foot IDs and exclude hardware/grouped skull.
2. Trace foot sections/stations to `profiles/f28.json#/foot_proxy_layout` and
   `FOOT_PROXY_LAYOUT.md`. Keep all 52 source dimension dictionaries unknown until
   measured evidence is adopted; do not treat display envelopes as osteometry.
3. Next bounded coverage PR: skull identities, with missing facial source records
   explicitly accounted for. Ear/hyoid follow separately. Dimensional/morphological
   adoption and working joints require separate evidence.
4. Preserve deterministic Vitruvian-derived canon 1.0.0 exactly: H = span =
   1676.4 mm. Scale, materials and mechanisms may change; canon does not drift.
   A0's 317 mm dummy station remains separate and project-local.
