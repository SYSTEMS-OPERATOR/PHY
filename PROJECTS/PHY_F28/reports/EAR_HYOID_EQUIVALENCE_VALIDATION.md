# Individual ear and hyoid bone-equivalence / validation checkpoint

Reviewed 2026-10-08 UTC against PHY main
`49c440cfcc5143c7d9bad338bada4553bfbdaaf1` (merged PR #102).

Status: **206/206 individual project identities represented**. Every new station,
section and surface remains provisional display geometry. Dimensional fidelity is
unverified, physical evidence unmeasured and fabrication release false.

## Coverage

| Disposition | Before | This checkpoint |
| --- | ---: | ---: |
| Individual project proxies | 199 | 206 |
| Individually represented ear ossicles | 0 | 6 / 6 |
| Individually represented hyoid | 0 | 1 / 1 |
| Grouped identities | 0 | 0 |
| Unrepresented identities | 7 | 0 |
| Expected adult inventory identities | 206 | 206 |
| Canonical source records present | 193 | 193 |
| Missing facial source records | 13 | 13 |

Each ear has a separate malleus–incus–stapes chain in lateral-to-medial display
order. Three left counterparts reflect authored right vertices, triangle winding,
bounds, anchors and side-specific topology/housing IDs. Stapes sweeps preserve an
open stirrup center. Temporal housing and tympanic-membrane/oval-window endpoints
are labels; cavities, joint interfaces, footplate morphology and hearing mechanics
are unresolved. The hyoid is one symmetric open U below the jaw, with posterior
ends and no direct bone articulation. Muscle/ligament suspension and swallowing
mechanics remain unmodeled.

`profiles/f28.json#/ear_hyoid_proxy_layout` governs every new geometry input.
Layout SHA-256: `c28df92e058ccd8b50672df576eb09c0830dd4c744ccb3eac4401f1ebe5ab6b2`.
`EAR_HYOID_PROXY_LAYOUT.md` lists all parameters and primary topology references.
External head breadth/depth and proposed head height define a display frame;
local millimeter choices scale from the F28 mean stature. None are measured bone
dimensions or adopted canon. All seven existing canonical source modules retain
their legacy dimension dictionaries and classifications unchanged.

## Local verification

Python 3.12.14: **154 focused pytest checks passed**, with no skipped tests:

```sh
PYTHONPATH=. python3 -m pytest \
  tests/test_ear_hyoid_geometry.py tests/test_skull_bone_geometry.py \
  tests/test_fabrication_validation.py tests/test_foot_bone_geometry.py \
  tests/test_hand_bone_geometry.py tests/test_bone_equivalence.py \
  tests/test_phy_studio.py tests/test_sophy_geometry_canon.py \
  tests/test_sophy_project_canon_bindings.py tests/test_reference_dimensions.py \
  tests/test_shoulder_dimensions.py tests/test_t56_geometry_contracts.py \
  tests/test_t56_a0_scope.py tests/test_t56_a0_geometry.py \
  tests/test_t56_a0_validation.py tests/test_component_3d.py -q
```

Nine new tests independently check the seven IDs and complete 206-ID inventory,
same-side ear order/topology, non-bone endpoints, symmetric unarticulated hyoid,
exact reflection and source paths, finite nondegenerate triangles, connected
closed oriented surfaces, positive signed volume, the stapes mesh's single hole,
uniform scaling across heights/poses, traceable hashes and lost-identity detection
without changing total mesh count. Thirty malformed profile mutations fail with
`ValueError`. These checks do not qualify anatomical surfaces, self-intersections,
mesh contact, joint fits, continuous clearance or structural performance.

All **223 previous parts** are exactly equal to the captured merged-main model.
The complete serialized hash for its 199 individual bone proxies remains:
`5d8610aaee83e5d487739c66818af8c73cc22ee1eca2b49810141766f9f8f6cc`.
Every original profile section is unchanged; CORE_V1, HAND_V1, FOOT_V1 and SKULL_V1
reports remain byte-identical. Canonical BoneSpec records, authority/schema files,
pinned measurements, SOPHY canon 1.0.0 and bindings, and the separate A0-R1 article
are preserved. H = span = 1676.4 mm remains the SOPHY comparison scale, and A0's
317 mm station remains separate. All six passive maquette files are byte-identical.

Node 24.19.0: `npm run check`, `npm run build` and `npm run test:logic` pass.
The actual Three.js export regression verifies 206 unique bone IDs, all seven
ear/hyoid hashes and parameter pointers, present source records, reflected ear
chain/housing metadata, non-bone endpoints, empty hyoid articulations and false
motion status. It also checks search/role filtering, GLB structure/extras and a
1700 mm canonical Z-up STL. Bone-only exports exclude hardware and form guides.
The existing 13 missing facial source records remain explicit in GLB extras.

The five-model export has 230 parts per reference overlay, 35 separate A0
instances and 18 source-gallery components: **743 viewable parts** total.
A fresh full export repeats all 24 output files byte-for-byte, including a valid
23-entry ZIP; `ZipFile.testzip()` reports no corrupt entries. The checked-in
offline HTML and all seven generated coverage/layout reports match the package.
The standard-library three-model packages also repeat exactly in pytest.
Local visual inspection confirms the stirrup openings, posterior hyoid ends and
separation below the mandible; it is a display review, not anatomical validation.

## Checks left to CI and physical work

Local Chromium and Python 3.11 are unavailable, so browser/WebGL interaction and
Python 3.11 compatibility are not claimed locally. The browser smoke suite now
checks ear/hyoid search, source status, visibility and GLB metadata. The Studio
workflow runs the new Python tests and compares the generated ear/hyoid report.
Remote CI is not awaited before opening this PR; inspect its latest-head results
before merge. No full-repository local test run or physical build is claimed.

## Review and next scope

Search `BONE_MALLEUS_R`, `BONE_INCUS_L`, `BONE_STAPES_L` and `BONE_HYOID` in Studio.
Check individual-bone-only viewing/export contains 206 IDs and keeps the seven
new source statuses and EAR_HYOID_V1 provenance. Trace the governing inputs to
`EAR_HYOID_PROXY_LAYOUT.md` and verify all three qualification flags remain false.

The identity/distribution gap is closed. The next separate work is the 13 missing
facial source records, source classification/evidence review, measured morphology
and functional joint interfaces. Count completion does not adopt measurements or
qualify fabrication, hearing, swallowing or any working joint.
