# Bone-equivalence checkpoint — individual skull

Base: PHY main `b96dfc25ae39fcba00046f827fe4e822e75f3fcc` (merged PR #101).
Scope: 22 individually identified project-local skull display proxies. This
closes skull identity/distribution coverage. Dimensions, surfaces, sutures and
functional jaw mechanics remain unverified.

## Confirmed deviations

| ID | Governing source at base | Confirmed condition | Correction / regression |
| --- | --- | --- | --- |
| BS-01 | `core_bone_geometry.build_core`, `head_arch_0` classification | One head arch corresponds to all 22 skull identities | Eight cranial and 14 facial identities each get one inspectable mesh; form arches/bands carry no bone IDs |
| BS-02 | `studio_model.build_armature`, head-form loop | Mandible and paired facial regions have no distinct geometry | Separate symmetric adult jaw, mirrored paired geometry, septum/face/vault ordering and selected adjacency checks |
| BS-03 | `bone_equivalence.source_record_paths` | 13 facial source records are absent; nine skull records have legacy dimensions without qualified morphology | Preserve all source records; expose missing records in audit, inspector and GLB extras; no invented measurements |
| BS-04 | Studio coverage, filtering and exports | The review describes 177 individuals and 22 grouped skull identities | Update to 199 individuals, zero grouped identities and seven unrepresented ear/hyoid identities; verify actual GLB identity/provenance |

## Inputs and authority

One `skull_proxy_layout` section (`SKULL_V1`) governs every new shape and
placement. The existing pinned ANSUR external head breadth and anterior/posterior
head length size a display frame; the proposed head height remains 220 mm.
None is an individual bone measurement. Shell inset/thickness and sweep sections
are explicit provisional choices, uniformly scaled from cohort mean stature.
Every skull mesh carries geometry pointers, a layout hash, unverified dimensional
fidelity and unmeasured physical evidence.

Six midline and eight right-side meshes are authored. Left vertices, bounds,
winding, anchors and selected adjacency IDs reflect the right. Frontal, parietal,
occipital and temporal regions use capped shell patches; sphenoid, ethmoid and
several facial regions use ellipsoids. Zygomatic and inferior nasal concha regions
use short sweeps. The single adult mandible is an open U-shaped sweep with
superior posterior ends and two temporal placement stations. Those stations
are not measured condyles, joint axes or qualified interfaces. No jaw motion is
implemented.

The identity and coarse distribution reference is OpenStax,
[Anatomy and Physiology 2e, 7.2 The Skull](https://openstax.org/books/anatomy-and-physiology-2e/pages/7-2-the-skull),
reviewed 2026-10-07 UTC. It supports eight cranial and 14 facial identities,
upper ethmoid/lower vomer septum, anterior maxilla/posterior palatine regions,
medial lacrimal/lateral zygomatic regions and the adult mandible's temporal
articulation. It supplies no numeric dimensions for this profile. Selected
adjacency labels are not a complete suture/articulation graph or a mesh contact
proof. PHY remains the governing project authority.

## Bounded closure

Result: **199 individual proxies**, zero grouped identities and seven
unrepresented ear/hyoid identities, still 206 in all. All 193 canonical records
are unchanged; the 13 missing facial records remain missing. Sutures, foramina,
sinuses, teeth, cartilage, measured morphology, contact, clearances, jaw motion,
loads and physical qualification remain open. Closed positive-volume display
meshes do not establish manufacturability or anatomical fidelity.

Preserve the previous 177 proxies exactly, SOPHY canon 1.0.0, H = span = 1676.4 mm,
the separate A0-R1 article, authority/schema files, source snapshots and the
byte-identical passive 26-cut maquette. Keep fabrication release false.

Review independent 22-ID coverage, reflected geometry and topology, selected
placement relationships, connected/oriented watertight meshes, rejected profile
inputs, scaling, actual GLB extras, repeated packages and preserved artifacts.
Record local results and skipped checks in `reports/SKULL_EQUIVALENCE_VALIDATION.md`.
Open the focused PR without waiting for long remote smoke CI; do not auto-merge.
The next coverage scope is six ear ossicles and the hyoid. Measured morphology
and functional joints require separate evidence and adoption.
