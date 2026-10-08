# Individual skull bone-equivalence / validation checkpoint

Reviewed 2026-10-07 UTC against PHY main
`b96dfc25ae39fcba00046f827fe4e822e75f3fcc` (PR #101).

Status: **bounded identity/distribution review**. Every new skull surface and
station is provisional project geometry. Dimensions and morphology are
unverified, physical evidence unmeasured and fabrication release false.

## Coverage

| Disposition | Before | This checkpoint |
| --- | ---: | ---: |
| Individual project proxies | 177 | 199 |
| Individually represented skull bones | 0 | 22 / 22 |
| Grouped skull identities | 22 | 0 |
| Unrepresented ear/hyoid identities | 7 | 7 |
| Expected adult identities | 206 | 206 |
| Canonical source records present | 193 | 193 |
| Missing facial source records | 13 | 13 |

Eight cranial and 14 facial identities are inspectable, including one symmetric
adult mandible. Eight paired counterparts reflect authored right geometry,
triangle winding, bounds, anchors and selected adjacency IDs. The five retained
head arches/bands are form-study meshes with no bone correspondence. Selected
adjacencies are reciprocal and connected, but do not define a complete suture,
articulation, clearance or load-path graph.

`profiles/f28.json#/skull_proxy_layout` governs all new shape inputs.
Layout SHA-256: `ad6a9ccff22eda308e6262af399217596d5ef67d6a874d2678928b8895115ddc`.
`SKULL_PROXY_LAYOUT.md` lists the parameters and source distinction. External
head breadth/depth and proposed 220 mm head height define a display frame,
not adopted osteometric dimensions. Nine skull identities have legacy source
records; 13 facial records remain missing in the audit, inspector and GLB extras.

## Local verification

Python 3.12.14: **145 focused pytest checks passed**, with no skipped tests:

```sh
PYTHONPATH=/workspace/scratch/360477491497/test-deps:. python3 -m pytest \
  tests/test_skull_bone_geometry.py tests/test_fabrication_validation.py \
  tests/test_foot_bone_geometry.py tests/test_hand_bone_geometry.py \
  tests/test_bone_equivalence.py tests/test_phy_studio.py \
  tests/test_sophy_geometry_canon.py tests/test_sophy_project_canon_bindings.py \
  tests/test_reference_dimensions.py tests/test_shoulder_dimensions.py \
  tests/test_t56_geometry_contracts.py tests/test_t56_a0_scope.py \
  tests/test_t56_a0_geometry.py tests/test_t56_a0_validation.py \
  tests/test_component_3d.py -q
```

The eight new skull tests independently check 22 IDs, eight cranial/14 facial
distribution, all 13 missing records, coarse vault/septum/face/jaw ordering,
reciprocal selected topology, exact reflected meshes, finite nondegenerate
triangles, connected closed oriented surfaces, positive volume, uniform scaling,
provenance hashes and lost-identity detection. Thirty-one malformed profile
mutations fail with `ValueError`. These are display-mesh checks, not an anatomical
surface, self-intersection, joint-contact or structural qualification.

The previous 177 individual proxies retain their complete serialized part hash
from merged main:
`44debc67ab8391f2efc9fa982d50dfa719f8d9e87c6e708b38d6cee306fdcce7`.
The separate A0-R1 assembly, SOPHY canon 1.0.0 and bindings, canonical BoneSpec
records, authority/schema files, hand/foot geometry modules and pinned source
snapshots are unchanged. All six passive maquette files remain byte-identical.

`npm run check`, `npm run build` and `npm run test:logic` pass. The actual Three.js
export regression checks 199 unique bone IDs, all 22 skull layout hashes and
input pointers, 13 explicit missing source records, the mandible's two temporal
neighbors and false motion status, role filtering/search, GLB structure/extras
and a 1700 mm Z-up STL. Form guides and hardware are excluded from bone-only
exports. JSON retains the complete source model and all 206 dispositions.

The five-model export has 223 parts per reference overlay, 35 separate A0
instances and 18 source-gallery components: 722 viewable parts total.
A fresh full export repeats every package file byte-for-byte, including a valid
22-entry ZIP; `ZipFile.testzip()` returns no corrupt entries. The checked-in
offline HTML and all six generated coverage/layout reports match the package.
The standard-library three-model packages also repeat exactly in pytest.

## Checks left to CI and physical work

Local Chromium and Python 3.11 are unavailable, so browser/WebGL interactions
and Python 3.11 compatibility are not claimed locally. The browser smoke suite
now covers skull selection, source-gap display and skull GLB extras. The Studio
workflow includes the new Python tests and repeated skull-layout report check.
Remote CI is not awaited before opening this PR; review its latest-head results
before merge. No full-repository local test run or physical build is claimed.

Sutures, foramina, sinuses, teeth, cartilage, measured skull morphology, jaw
articular surfaces/axes/motion, contact and continuous clearances remain open.
Closed positive-volume meshes and count equivalence cannot qualify fabrication,
dimensional fidelity or working joints. No canonical measurements are adopted.

## Review and next scope

Search `BONE_MAXILLA_R`, `BONE_PAR_L` and `BONE_MANDIBLE` in Studio. Check
bone-only viewing and GLB exports retain those identities, source status and
`SKULL_V1` provenance while excluding the head form guides. Trace the provisional
stations to `SKULL_PROXY_LAYOUT.md`; missing facial records stay missing.

Next bounded coverage PR: six ear ossicles and the hyoid, bringing individual
coverage to 206/206. Measured morphology and functional joints follow through
separate evidence and adoption. Preserve H = span = 1676.4 mm and canon 1.0.0.
