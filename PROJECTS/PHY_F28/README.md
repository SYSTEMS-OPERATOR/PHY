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
- Internal thoracic formers, pelvic arches, external hip/chest form rails and the
  interpolated soft-form envelope are proposed supports, not human scan data.

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
It is a separate construction embodiment; it does not miniaturize A0 bearings or
claim full-scale strength. Physical build and dimensional inspection are unmeasured.

## Reproduce and validate

```bash
python bin/export_phy_studio.py
python -m pytest tests/test_phy_studio.py -q
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
