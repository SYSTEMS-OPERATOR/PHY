# Individual hand bone-equivalence / validation checkpoint

Reviewed 2026-10-06 UTC against PHY main
`92ecbd3ec77f4fb74fbd708af91973a947ce1c8f` (PR #98).

Status: **bounded identity/distribution review, not dimensional equivalence or
fabrication release**. Every new section, station and gap is project-local
provisional geometry. Physical evidence is unmeasured. Canon 1.0.0 and the
separate A0-R1 article remain unchanged.

## Resulting coverage

| Disposition | Before | This checkpoint |
| --- | ---: | ---: |
| Individually identified project proxies | 71 | 125 |
| Grouped skull/hand/foot identities | 128 | 74 (skull/feet) |
| Unrepresented ear/hyoid identities | 7 | 7 |
| Individually represented hand bones | 0 | 54 / 54 |
| Expected adult identities | 206 | 206 |
| Missing facial source records | 13 | 13 |

Each hand has eight separate carpals, five metacarpals and 14 phalanges, including
two thumb phalanges and three per other digit. Two carpal rows, radial/ulnar
order, the palmar pisiform and a separately angled radial/palmar thumb chain are
explicit. Selected chain-parent labels are partial topology, not a complete
joint/contact graph. The static thumb pose is not working opposition.

The old palm/finger fan generators and their grouped correspondence are replaced;
canonical BoneSpec modules and the preserved 26-cut passive maquette are not
replaced. The hand profile lists every governing input and has its own hash.
Mirrored left meshes carry reflected vertices, reversed winding, reflected
stations/anchors and left-qualified topology/provenance.

## Executed checks

| Check | Result / limits |
| --- | --- |
| Python 3.12.14 focused engineering suite | 110 tests and 431 subtests passed; no skips |
| Python 3.11.16 bone/hand/Studio/canon/bindings suites | 43 tests and 194 subtests passed; one optional CadQuery viewer test skipped in the isolated environment |
| Identity and failure detection | Exact independent 27-bone lists per side; hand source correspondence; lost thumb identity remains visible despite unchanged mesh count |
| Frame/topology | Both sides at 0°, 45° and 80°, using cohort/1676.4/1800 mm overlays; carpal row and radial order, palmar pisiform, radius-side angled thumb, segment continuity and middle-finger station closure |
| Parameters and evidence | Reject missing carpals, wrong rows/root/segment counts, negative/nonfinite sizes, invalid vectors/order, ulnar thumb, dorsal pisiform and invalid/consuming gaps; uniform section/station/gap scaling; unverified/unmeasured flags and input hash checks |
| Meshes and reflection | Exact reflected vertices/winding, anchors, topology and provenance; all 151 reference meshes are watertight, nondegenerate and positive-volume |
| Node syntax/build | Checks and offline bundle rebuild passed |
| Actual Three.js exports without WebGL | 125 unique bone IDs in GLB, 54 hand identities with layout hash/input pointers/evidence, mirrored thumb parent, hand-ID search, hardware/group exclusions and explored 1700 mm complete-frame STL passed |
| Full repeat/cross-version package and protected-file checks | Pending validation checkpoint |

An initial frame-test harness expected a forearm `center_mm` field that those
proxies do not expose; it now checks their actual distal endpoints. All corrected
frame cases pass. This did not require changing the forearm geometry.

## Unrun / unresolved

- Local browser smoke failed to launch: the pinned Chromium headless executable
  is absent. No new local WebGL, surface-selection, visual/mobile or interaction
  pass is claimed. The extended remote Studio workflow checks 125-bone coverage,
  individual thumb search and hand GLB metadata before merge. Do not wait for its
  long run to open the PR or auto-merge it.
- No local full-repository pass claimed; the focused suite does not cover every
  runtime subsystem. The Python-package CI remains responsible for that scope.
- Bone morphology, measured endpoints, articular surfaces, carpal/wrist axes,
  functional thumb opposition, tendons/ligaments and working connections are
  unresolved. Display gaps are not qualified fits; no interference/motion proof,
  structural analysis, machining STEP qualification, fabrication or physical
  testing is asserted for these meshes.

## Independent review and next bounded work

1. Confirm latest-head CI passes. In Studio, search `BONE_PHAL_1_2_R`, inspect
   both hands and verify individual-only GLB/STL exports retain hand identities.
   Inspect the palmar view and a side view to distinguish the pisiform and thumb
   palmar offset; an overlap in palm projection alone is not a physical contact.
2. Trace each hand input to `profiles/f28.json#/hand_proxy_layout` and
   `HAND_PROXY_LAYOUT.md`. Keep proposed display geometry separate from unknown
   BoneSpec measurements. NASA/ANSUR external envelopes do not supply bone lengths.
3. Next bounded coverage PR: 26 individual foot identities per side. Cranial,
   facial, ear and hyoid identities follow separately. Dimensional/morphological
   adoption and physically functional joints require their own measured evidence.
4. Preserve deterministic Vitruvian-derived canon 1.0.0 exactly: H = span =
   1676.4 mm. Scale, materials and mechanisms may change; canon does not drift.
   A0's 317 mm dummy station remains separate and project-local.
