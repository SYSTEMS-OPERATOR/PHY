# PHY F28 — adult female reference and first physical form study

This project creates a complete visual armature and a quarter-scale supported
maquette. It gives PHY an inspectable whole-body reference and an inexpensive
physical demonstration while functional joint engineering continues in A0-R1.

## Evidence and authority

`references/ansur_age28.json` is a hash-pinned projection of the same ANSUR II public
CSV already selected by T56. The source CSV SHA-256 matches the existing catalog.
Every overlapping field and all 92 row references match T56's independent snapshot.
Selection is **female and age exactly 28**, with no height, BMI, race or appearance
filter. Statistics are arithmetic means and sample standard deviations.

ANSUR II measured US Army personnel in 2010–2012. This is a specified reference
population, not the current worldwide or US civilian average. Marginal means do not
describe one actual woman. No claim of a unique age-28 anatomy or objective beauty
optimum is made.

The baseline mean height is **1632.402 mm**; measured span is **1664.359 mm**.
The refined profile keeps those dimensions and changes shoulder breadth −1%, waist
form −3%, and hip form +2%. Changes never overwrite measured source data.

## Explicit mechanical design choices

The mechanical model is a form-study representation, not a new BoneSpec schema.
It leaves canonical bone records, BODY authority, SOPHY canon 1.0.0, and A0 geometry
unchanged. It is not a replacement for their fabrication validators.

- External surface dimensions are not bone lengths or joint-center distances.
- Proposed head height is 220 mm; ANSUR head length is front-to-back depth.
- Shoulder centers sit 14 mm medial to each acromion; hip-center spacing is 0.68
  times bicristal breadth, 35 mm above the trochanterion reference height.
- Knee height uses lateral epicondyle height as a station proxy; ankle datum is
  70 mm. Joint coupling solids are envelopes, not bearing or retention designs.
- The arm chain is allocated from external segment ratios to close the measured
  span after subtracting proposed shoulder-center spacing. This endpoint mapping
  is explicit and is not called a slight aesthetic change or measured osteometry.
- External hip/chest form rails and the interpolated envelope remain non-bone
  supports/form guides, not human scan data. Hardware couplings do not count as bones.

## Bone distribution / 206 individual identities

`reports/BONE_EQUIVALENCE.md` accounts for all 206 adult identities, independently
of mesh counts. All 206 bones now have individual proxies, including 71 core bones (26 spine, 25 thorax,
four shoulder-girdle, two adult hip, six upper-limb, eight lower-limb). Ribs are
24 separate open curves; cervical/thoracic/lumbar levels remain individually
identified, paired forearms have radius and ulna, and patellae are separate from
knee coupling envelopes. Adult fused hip/sacrum/coccyx identities count once each.

Another 54 individual hand proxies provide eight carpals, five metacarpals and
14 phalanges per side. The radial thumb chain is separate and palmar-offset;
carpal rows and the palmar pisiform are distinguished. This is a static display
pose, not working opposition, a complete articulation graph or measured bone shape.

Another 52 foot proxies provide seven tarsals, five metatarsals and 14 toe
phalanges per side. The heel/talus and medial hallux are distinct; cuneiforms
and cuboid are named chain parents. These are static grounded display proxies,
not measured tarsal shapes, functional arches or working ankle/toe mechanics.

Another 22 skull proxies provide eight cranial and 14 facial identities, including
one separate adult mandible. Vault patches, facial ellipsoids and sweeps are
provisional distribution geometry. Sutures, foramina, sinuses, teeth and jaw
mechanics remain unresolved; temporal stations do not implement a joint.

The final seven proxies provide a malleus–incus–stapes chain on each side and one
midline hyoid below the jaw. Stapes meshes preserve an open stirrup center; the
hyoid has posterior ends and no direct bone articulation. Temporal housing and
tympanic-membrane/oval-window endpoint labels do not establish cavities, fits,
hearing or motion. Hyoid suspension and swallowing mechanics remain unmodeled.
No grouped or unrepresented inventory identities remain.
13 facial source records remain absent; the 193 existing
canonical records are untouched. The `bone_proxy_layout` profile section is the
authoritative project-local provisional core layout; `hand_proxy_layout` governs
the hands, `foot_proxy_layout` the feet, `skull_proxy_layout` the skull and
`ear_hyoid_proxy_layout` the final seven proxies. All five have hashes and parameter pointers
embedded in the model. It is **not measured bone shape, joint anatomy or canon**.
Count/distribution equivalence alone cannot establish dimensional or kinematic fidelity.

Studio's Bone coverage panel exposes every disposition and its source record.
Individual-bone-only viewing/export hides hardware, supports and head form guides.
See `BONE_EQUIVALENCE_SCOPE.md` for confirmed deviations and bounded corrections.
The hand scope and historical checks are in `HAND_EQUIVALENCE_SCOPE.md` and
`reports/HAND_EQUIVALENCE_VALIDATION.md`; `reports/HAND_PROXY_LAYOUT.md` lists
every governing hand input. Foot scope/checks are in
`FOOT_EQUIVALENCE_SCOPE.md` and `reports/FOOT_EQUIVALENCE_VALIDATION.md`;
`reports/FOOT_PROXY_LAYOUT.md` lists governing foot inputs. Earlier checkpoints
remain historical. All 52 foot source dimension dictionaries remain unknown.
Skull scope/checks are in `SKULL_EQUIVALENCE_SCOPE.md` and
`reports/SKULL_EQUIVALENCE_VALIDATION.md`; `reports/SKULL_PROXY_LAYOUT.md` lists
every governing skull input. Studio's skull inspector and GLB extras explicitly
mark canonical source records as present or missing.
Current final-seven scope/checks are in `EAR_HYOID_EQUIVALENCE_SCOPE.md` and
`reports/EAR_HYOID_EQUIVALENCE_VALIDATION.md`; `reports/EAR_HYOID_PROXY_LAYOUT.md`
lists every governing ear/hyoid input. All seven already had canonical source
modules; their legacy dimensions and classifications remain unchanged.

The SOPHY-scale comparison chooses H=span=1676.4 mm. It does not adopt new canon
landmarks. A0's separate 317 mm dummy station is neither scaled nor integrated
automatically into this whole-body reference.

## Demonstrate and make

```bash
python bin/phy_studio.py
```

Select Mean, Refined or SOPHY Scale, change view, isolate a part, inspect its source,
and export the frame. The Studio also includes the real A0-R1 neutral CAD assembly
and the prior 18-component source library.

`maquette/` contains actual-size SVG cut paths, an assembly stencil, part/hardware
BOM and instructions for a **408.10 mm tall passive plywood maquette**. It uses a
rear support post, solid transverse formers and simple pinned planar limb links.
It is a separate, preserved construction embodiment; it does not miniaturize the
new core bone proxies or A0 bearings or claim full-scale strength. Physical build
and dimensional inspection are unmeasured. Its seven torso formers are not ribs.

## Reproduce and validate

```bash
python bin/export_phy_studio.py
python -m pytest tests/test_bone_equivalence.py tests/test_phy_studio.py -q
```

The exporter writes complete reference meshes, their source report, cut files and
offline HTML to `studio/dist/`. CadQuery 2.7.0 is required for A0/library tessellation;
`--without-a0` uses only the Python standard library and the checked-in JS bundle.

Tests cover source hash/field/cohort agreement, mirrored geometry, height/span
closure, nondegenerate watertight positive-volume meshes, input/refinement bounds,
maquette file and BOM integrity, and unchanged A0 located B-rep bounds.

## Next functional release

The visual reference is demonstrable. A full-size functional armature still needs
joint internals, retained motion, interfaces, load cases, stability/support,
fabrication drawings and inspection. A0 currently has a 3.25 kN bearing reaction
screen against an unverified 3.0 kN threshold; supplier correlation, impact-pad
response, independent review, continuous clearance and bench evidence remain open.
