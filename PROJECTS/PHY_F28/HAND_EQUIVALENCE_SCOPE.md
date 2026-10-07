# Bone-equivalence checkpoint — individual hands

Base: PHY main `92ecbd3ec77f4fb74fbd708af91973a947ce1c8f` (merged PR #98).
Scope: 27 individually identified project-local hand proxies per side, authored
on the right and reflected on the left. This closes hand **identity/distribution**
coverage, not measured bone dimensions, anatomical surfaces or working joints.

## Confirmed deviations

| ID | Governing source at base | Confirmed condition | Correction / regression |
| --- | --- | --- | --- |
| BH-01 | `studio_model.build_armature`; `bone_equivalence.bone_audit` | Eleven generic hand meshes per side account for 27 grouped identities | Eight carpals, five metacarpals and 14 phalanges per side; exact IDs, counts and role tests |
| BH-02 | `studio_model.build_armature` digit loop | Digit 1 is the negative-lateral member of a generic fan; the radius proxy is positive-lateral | Thumb on the radius side with a separately angled, palmar-offset chain; pose/frame tests |
| BH-03 | `core_bone_geometry.build_core` palm grouping | Eight carpals and five metacarpals share one carrier, with no carpal rows | Two named rows, palmar pisiform relative to triquetrum and trapezium-to-thumb relationship; topology/position tests |
| BH-04 | Existing hand BoneSpec modules | Their dimensions and articulations do not establish measured hand geometry | Preserve source records. Fractional placement, sections and gaps remain explicitly provisional; reject invalid inputs |
| BH-05 | Studio role/export regressions | Existing expected totals enforce the previous 71-bone checkpoint | Update exact independent totals and test hand-ID search and actual GLB metadata; do not weaken exclusions |

## Governing inputs and authority

- Reuse the pinned ANSUR external hand breadth and the existing span-derived hand
  station. Neither provides individual bone length or a true wrist joint center.
- One new `hand_proxy_layout` section in `profiles/f28.json` governs placement,
  positive sections and nominal display gaps. Every individual hand proxy carries
  input pointers, a layout hash and unverified/unmeasured evidence status.
- Use a wrist-relative orthonormal frame: distal along the arm, radial on the
  radius side, palmar toward subject anterior. Apply the same construction to
  A/T poses and uniformly scaled comparison overlays. Mirror geometry, winding,
  positions and side-qualified topology from the authored right side.
- The thumb contains metacarpal 1 plus proximal/distal phalanges; digits 2–5 each
  contain one metacarpal and three phalanges. The thumb placement is a static
  review pose, not a functioning opposition mechanism or a kinematic claim.
- Preserve canon 1.0.0 exactly, H = span = 1676.4 mm. Do not edit BoneSpec,
  BODY/MIND/SOUL, canonical geometry or the separate A0-R1 article. The passive
  26-cut maquette retains its earlier schedule and exact files.

The already selected OpenStax [upper-limb source](https://openstax.org/books/anatomy-and-physiology-2e/pages/8-2-bones-of-the-upper-limb)
(reviewed 2026-10-06 UTC) supports hand identities, carpal row relationships,
palmar pisiform, radial thumb and phalanx counts. It supplies **no numeric values**
for this profile. No new anatomical measurement/surface dataset is imported.

## Closure and remaining work

Expected review totals: **125 individual proxies**, 74 grouped skull/foot
identities and seven unrepresented ear/hyoid identities, still 206 in all.
Keep missing source records, dimensional fidelity, articular surfaces, joint axes,
motion, interference, strength and physical evidence visibly unresolved. The
intentional display gaps are not qualified fits or a functional assembly.

Checkpoint commits: bounded audit; right-hand geometry and mirror; validation
and regenerated Studio packet. Run both Python 3.11/3.12 regressions, exact
maquette checks, independent regional identity checks, positive-volume meshes,
actual Three.js export checks and repeat-package comparisons. Record any skipped
browser/physical checks; open a PR without waiting for the long remote smoke
workflow and do not auto-merge. Next coverage scope is individual feet.

## Checkpoint dispositions

BH-01–03 and BH-05: implemented for bounded identity/placement review and tested
locally. Both hands have exact independent 27-ID coverage and right-derived
left geometry; the offline packet and actual GLB metadata expose each identity.
Remote browser/full-package CI remains required before merge. BH-04 remains
open for dimensional/morphological/functional fidelity: all 54 hand BoneSpec
records still have wholly unknown dimensions, and no articular interface or
working mechanism is adopted. Results and skipped checks are in
`reports/HAND_EQUIVALENCE_VALIDATION.md`.
