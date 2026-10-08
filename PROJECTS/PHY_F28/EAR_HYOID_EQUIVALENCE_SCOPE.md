# Bone-equivalence checkpoint — ear ossicles and hyoid

Base: PHY main `49c440cfcc5143c7d9bad338bada4553bfbdaaf1` (merged PR #102).
Scope: six individual ear ossicles and one hyoid display proxy, completing
the project's adult 206-identity inventory. This is identity/distribution
coverage, not adopted morphology, working hearing/swallowing or fabrication.

## Confirmed deviations

| ID | Governing source at base | Confirmed condition | Correction / regression |
| --- | --- | --- | --- |
| BEH-01 | `bone_equivalence.expected_bones`, reference model audit | Six ear ossicles and the hyoid are unrepresented | Three individual ossicles per side and one midline hyoid; independent seven-ID and complete 206-ID checks |
| BEH-02 | Existing malleus/incus/stapes source modules | Each uses the same legacy 8 × 4 × 3 mm box envelope; no qualified articular/placement geometry | Provisional tapered/curved sweeps and a stirrup loop, authored on the right and reflected on the left; preserve all source data |
| BEH-03 | Existing hyoid source module and head display | A source box exists but there is no separate suspended U-shaped display | One open U below the mandible with posterior ends; no invented bone articulation, axis or muscle/ligament mechanism |
| BEH-04 | Studio filtering, coverage and exports | Review expectations stop at 199 individuals and seven unrepresented identities | Update to 206 individuals, zero grouped/unrepresented; preserve source-gap and fabrication blockers and actual GLB provenance |

## Governing inputs

One `ear_hyoid_proxy_layout` profile section (`EAR_HYOID_V1`) governs every new
offset, centerline and section. Ear origins use fractions of the existing head
frame; local ossicle offsets/sections and hyoid centerline offsets use provisional
mm at cohort mean stature, uniformly scaled for overlays. External ANSUR head
breadth/depth and proposed 220 mm head height do not measure individual bones or
joint centers. No legacy dimension is adopted as qualified morphology.

The right malleus → incus → stapes chain progresses laterally to medially in a
provisional review layout. Tapered sweeps distinguish malleus and incus; a
closed stirrup-like sweep distinguishes stapes. The temporal bone is a housing
reference, not an invented bone articulation. Tympanic-membrane and oval-window
endpoints are non-bone labels; cartilage and soft tissue do not increase coverage.
The left three meshes, winding, anchors and side-specific topology derive by
reflection. No acoustic transfer, joint fit, footplate surface or hearing
mechanism is implemented.

The hyoid is one symmetric midline U below the mandible, with its ends posterior
to the body. Its bone articulation list is empty. Muscle/ligament suspension,
tongue/larynx/pharynx interfaces and motion remain unmodeled.

Primary identity/coarse-topology references, reviewed 2026-10-08 UTC:

- [OpenStax Anatomy and Physiology 2e, 14.1](https://openstax.org/books/anatomy-and-physiology-2e/pages/14-1-sensory-perception): three middle-ear ossicles and malleus/incus/stapes transmission order.
- [OpenStax Anatomy and Physiology 2e, 7.2](https://openstax.org/books/anatomy-and-physiology-2e/pages/7-2-the-skull): temporal housing and a separate upper-neck hyoid with posterior ends and no bone articulation.

These sources provide no numeric profile inputs. PHY remains the governing
project authority. The proxy layout is not a measured articulation/contact graph.

## Closure and preserved boundaries

Expected result: **206 individually identified proxies**, zero grouped and zero
unrepresented identities. Canonical source completeness remains 193/206: all
13 absent facial records stay missing. Dimensional fidelity, physical evidence,
structural release and mechanism qualification remain open.

Preserve all previous 199 proxies, all 193 source modules, profile sections
CORE_V1/HAND_V1/FOOT_V1/SKULL_V1, pinned measurements, authority/schema files,
SOPHY canon 1.0.0 (H = span = 1676.4 mm), the separate A0-R1 article and the six
byte-identical passive maquette files. Fabrication release remains false.

Commit scope, implementation and validation as separate checkpoints. Verify
independent IDs, ear chain side/order, stapes loop, unarticulated hyoid, mirrored
metadata, connected oriented closed surfaces, scaling, rejected inputs, actual
GLB/STL exports and repeated full packages. Record skipped checks. Open the PR
without waiting for long remote smoke CI and leave merging to the user.

After identity closure, measured source/morphology adoption and functional joint
interfaces are the next separate engineering work; 206 meshes alone do not close
those gates.
