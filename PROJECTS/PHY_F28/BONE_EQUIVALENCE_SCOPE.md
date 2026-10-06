# Bone-equivalence checkpoint — core distribution

Base: PHY main `ae81de1cd3ac555a2ec57e540f9882a3a550adfe` (PR #97).
Scope: project-local anatomical correspondence and visual distribution, not
osteometric adoption, functional joints, machining CAD or fabrication release.

## Counting and authority

- Use the conventional adult 206-bone inventory (80 axial, 126 appendicular).
  Sacrum, coccyx and each adult hip bone count as one identity; fused subregions,
  cartilage, teeth, bearings, coupling envelopes and fascia are not extra bones.
- Preserve the 193 existing `BONE_*` IDs. Account for 13 missing facial IDs in
  the expected inventory without inventing BoneSpec measurements or modules.
- Every Studio mesh is classified as bone proxy, hardware, support, form or
  envelope. A grouped form does not count as individual bone coverage.
- Individually represent 71 core bones: spine 26, thorax 25, shoulder girdle 4,
  pelvis 2, upper arm/forearm 6 and thigh/shank/patella 8. The remaining 135
  identities remain individually accounted for, grouped or unrepresented.
- Project proxy positions and sections are proposed design, not measured bone
  lengths. Existing external ANSUR evidence and span allocation stay separate.
- SOPHY canon 1.0.0, H=span=1676.4 mm, and A0-R1 inputs/exports are immutable
  for this PR. Mirror left geometry from authored right geometry.
- The existing 26-cut maquette stays the earlier passive form study, not a
  miniaturization of the new bone-resolved reference. Its cut files remain unchanged.

## Confirmed discrepancies and dispositions

| ID | Source at base | Severity | Confirmed condition | Disposition / regression |
| --- | --- | --- | --- | --- |
| BE-01 | `studio_model.py:210–216` | high | 8 spine links + 8 couplings + one neck support, no level IDs | 7 cervical, 12 thoracic, 5 lumbar, sacrum and coccyx IDs; order test |
| BE-02 | `studio_model.py:229–238` | high | 7 closed thoracic rings, not paired ribs | 24 separate open rib curves at T1–T12; true/false/floating topology test |
| BE-03 | `studio_model.py:276` | high | One forearm rod per side | Separate radius/ulna; handedness and endpoint-separation test |
| BE-04 | `studio_model.py:244–258` | high | Scapular rails and pelvic form supports, no adult hip identity | Two scapular plate proxies and two hip proxies; girdle/mirror test |
| BE-05 | `studio_model.py:272–278` | medium | Knee coupling has no separate patella | Two anterior patella proxies; classification/position test |
| BE-06 | `studio_model.py:280–291` | high, deferred | 11 hand parts/side and one foot block/side | Explicit grouped correspondence to 27 hand and 26 foot IDs/side; never individual coverage |
| BE-07 | `studio_model.py:217–225`; `skeleton/bones/` | high, deferred | Head frame has no distinct jaw; 13 facial records absent | Skull grouping + explicit missing-record ledger; head/ear/hyoid work later |
| BE-08 | `studio_model.py:198–204` | high | No canonical bone identity on any mesh | Typed mesh roles, stable bone ID crosswalk; reject unknown IDs and duplicate coverage |
| BE-09 | `tests/test_phy_studio.py`; `test_bone_count_pytest.py` | high | Mesh closure/count tests do not enforce anatomical inventory or topology | Region-by-region exact identity tests, grouping/exclusion checks, mutation tests |
| BE-10 | `dist/skeleton_canonical.json` | high, unresolved | 157/193 records lack any positive numeric dimension | Keep dimensional fidelity unverified; do not turn proxy dimensions into BoneSpec truth |

## Sources and execution

Anatomical inventory/topology references (consulted 2026-10-06 UTC): OpenStax,
*Anatomy and Physiology 2e*, sections [7.1](https://openstax.org/books/anatomy-and-physiology-2e/pages/7-1-divisions-of-the-skeletal-system),
[7.2](https://openstax.org/books/anatomy-and-physiology-2e/pages/7-2-the-skull),
[7.3](https://openstax.org/books/anatomy-and-physiology-2e/pages/7-3-the-vertebral-column),
[7.4](https://openstax.org/books/anatomy-and-physiology-2e/pages/7-4-the-thoracic-cage),
[8.2](https://openstax.org/books/anatomy-and-physiology-2e/pages/8-2-bones-of-the-upper-limb),
[8.3](https://openstax.org/books/anatomy-and-physiology-2e/pages/8-3-the-pelvic-girdle-and-pelvis),
[8.4](https://openstax.org/books/anatomy-and-physiology-2e/pages/8-4-bones-of-the-lower-limb).
These support names, counts and relationships, not numeric proxy geometry. No
source artwork or anatomical surface dataset is imported.

Commit checkpoints: audit/inventory; core geometry/inspection; validation/package.
Deferred geometry is visible in the Studio coverage panel and exported JSON.
Continuous motion clearance, structural qualification and physical evidence
remain open. No new physical test or anatomical measurement is claimed.

## Checkpoint dispositions

BE-01–05 and BE-08–09: bounded core identity/distribution correction implemented;
71 individual project proxies and typed roles are tested. Bone morphology,
articular surfaces, axes and physically functional connections are not closed.
BE-06–07: grouped/deferred, not silently passed. The hand's existing generic digit
fan is not an opposed, anatomically lateralized thumb mechanism; assigning digit
IDs to it is bookkeeping only. Hand/foot/head/ear/hyoid individual geometry belongs
in later bounded PRs. BE-10 remains unresolved: no proxy section or station is
adopted into a missing BoneSpec measurement.

Validation results and explicit skipped checks are in
`reports/BONE_EQUIVALENCE_VALIDATION.md`. The preserved maquette is not evidence
that the new bone proxies assemble or realize joint motions.
