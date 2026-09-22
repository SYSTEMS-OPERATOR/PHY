# PHY source-based 3D component kit

This is a usable CAD review kit, **not a viable assembled T-5.6 or fabrication release**.
It contains 18 independently importable STEP solids and matching STL files:

- 13 rectangular biological reference envelopes from the local baseline dataset;
- one REDWOOD femur rough blank, 44 × 44 × 430 mm;
- four nominal brass bushings with modeled through-bores.

`exports/component_3d/viewer.html` is an offline interactive component viewer.
Open it after extracting the entire kit to keep the STEP/STL download links working.
STEP uses millimetres; select millimetres when importing the unitless STL format.
Each part has its own local frame. No catalog layout is an assembly arrangement.

## What the geometry means

The reference boxes encode three recorded dimensions, not anatomical contours,
cross sections, wood cutting instructions, mass models, or collision surfaces.
Generic Rib and Vertebra entries represent source averages, not individual bones.
Skull and Mandible are omitted because their differently named dimensions require
an explicit axis definition. Pin lengths and washer outer diameters/thicknesses
are absent, so those solids are omitted too.

The biological baseline has no source citations inside the dataset. Its boxes
are legacy references with file-level traceability, not independently validated
anatomy or canonical identity. REDWOOD remains at its own existing scale;
nothing is rescaled or adopted into T-5.6. The femur guide is provisional and
does not define drilling locations. Bushing nominal dimensions do not establish
fits, tolerances, loads, or manufacturing process adequacy.

## Rebuild

Use Python with CadQuery 2.7.0 installed, then run from the repository root:

```sh
python bin/export_component_3d.py
python -m unittest discover -s tests -p 'test_component_3d.py' -v
```

The generator uses only local source files. The HTML viewer needs no server,
package installation, or network. CadQuery is optional for the rest of PHY.
Source hashes, individual geometry classes, omissions, null assembly transforms,
and all 19 open T-5.6 inputs are recorded in `manifest.json`.
The manifest is deterministic for identical input files and CAD kernel.
STEP headers can contain export timestamps; byte-identical STEP files are not promised.

## Verification and remaining work

Each exported solid is checked for CAD validity, a single closed solid, exact
nominal bounding dimensions, analytic volume, and STEP re-import volume/validity.
These checks establish file and geometry integrity only.

An assembled model requires explicit adoption of anatomical landmark equations
through a versioned canon change, then dimensioned engineering datums and the
thorax, pelvis, shoulder, arm and clearance inputs in the T56 geometry register.
Scalar references cannot supply these missing coordinates. The next bounded
engineering article is the existing single-side shoulder fixture once its datum,
interface dimensions and mechanism choice are supplied. This kit does not change
the register or set any simulation/fabrication-ready flag.
