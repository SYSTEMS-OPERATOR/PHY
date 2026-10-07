# Bone-equivalence checkpoint — individual feet

Base: PHY main `214c503ff595576863b545a3e46891dd515f98ed` (merged PR #99).
Scope: 26 individually identified project-local foot proxies per side, authored
on the right and reflected on the left. This closes foot identity/distribution
coverage, not measured dimensions, anatomical surfaces or working joints.

## Confirmed deviations

| ID | Governing source at base | Confirmed condition | Correction / regression |
| --- | --- | --- | --- |
| BF-01 | `studio_model.build_armature`, foot-form loop; `core_bone_geometry.build_core`, grouped-foot classification | One ellipsoid per foot accounts for 26 grouped identities | Seven individual tarsals, five metatarsals and 14 toe phalanges per side; exact independent IDs and no grouped-foot accounting |
| BF-02 | Existing foot form and `bone_equivalence.expected_bones` | Inventory names exist but heel/talus, medial hallux and five toe chains cannot be inspected individually | Explicit right-foot frame, tarsal ordering and chain parents; mirrored positions, winding and topology tests |
| BF-03 | 52 existing foot `BoneSpec` modules | All 52 dimension dictionaries are wholly unknown; articulations do not define usable geometry | Keep all source records unchanged. Profile fractions, sections, elevations and display gaps remain provisional |
| BF-04 | Studio role/export regressions and generated packet | Expectations describe 125 individual proxies and 74 grouped identities | Update independent coverage and actual GLB/selection tests; preserve exclusions, maquette and canon checks |

## Governing inputs and authority

- Reuse the pinned external ANSUR heel-to-toe length and horizontal foot breadth,
  the existing ankle station, floor and uniform height scale. These describe a
  display envelope; they do not establish individual bone dimensions or joint
  centers. No new anatomical measurement/surface dataset is imported.
- One `foot_proxy_layout` profile section governs all new placement, sections,
  heel offset and gaps. Every foot mesh carries input pointers, a layout hash,
  unverified dimensional fidelity and unmeasured physical evidence.
- Right-foot frame: +x lateral, +y anterior, +z dorsal above the floor. Digit 1
  is medial, toward the subject midline; digit 5 is lateral. Talus is above the
  calcaneus, navicular anterior to talus, cuboid anterior to calcaneus, and the
  three cuneiforms are medial/intermediate/lateral anterior to navicular.
- Metatarsals 1–3 have cuneiform chain parents; 4–5 have the cuboid. The hallux
  has two phalanges, each other toe three. Parent labels describe partial
  topology, not complete articular contacts or a functional load path.
- Ellipsoids/tubes and elevated midfoot stations are display proxies, not
  reproduced tarsal shapes, qualified arches, fits, gait or contact surfaces.
  Uniformly scaled overlays and arm poses retain the same grounded foot frame.
- Preserve SOPHY canon 1.0.0 exactly, H = span = 1676.4 mm, and the merged core
  and hand layouts. Do not edit BoneSpec, BODY/MIND/SOUL, canonical geometry or
  the separate A0-R1 article. Preserve the passive 26-cut maquette byte-for-byte.

The already selected OpenStax [lower-limb source](https://openstax.org/books/anatomy-and-physiology-2e/pages/8-4-bones-of-the-lower-limb)
(reviewed 2026-10-07 UTC) supports identity counts, tarsal ordering, medial hallux,
cuneiform/cuboid-to-metatarsal relationships and phalanx counts. It supplies no
numeric values for this profile. PHY remains the governing project authority.

## Bounded closure and remaining work

Expected totals: **177 individual proxies**, 22 grouped skull identities and
seven unrepresented ear/hyoid identities, still 206 in all. Thirteen missing
facial source records remain missing. Dimensions, morphology, joints, contact,
motion, clearance, strength and physical evidence remain visibly unresolved.
Fabrication release, dimensional fidelity and physical qualification stay false.

Commit after this audit, geometry and validation. Check independent regional
IDs, tarsal positions, chain continuity/gaps, ground bounds, scaled geometry,
mirrored metadata, rejected inputs, actual Three.js exports, repeat packages,
Python 3.11/3.12 parity and unchanged canon/maquette. Record skipped checks.
Open a focused PR without waiting for long remote smoke CI; do not auto-merge.
Next coverage scope is skull, followed by ear/hyoid identities. Dimensional and
functional adoption requires separate measured evidence.
