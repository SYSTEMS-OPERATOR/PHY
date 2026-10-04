# PHY Studio

A portable, offline 3D workbench for PHY. Open `PHY-Studio.html` in a WebGL-capable
browser, or run from the repository root:

```bash
python bin/phy_studio.py
```

The app opens on `http://127.0.0.1:8765/PHY-Studio.html`. Use `--no-browser` on a
headless machine, or `--host 0.0.0.0` to deliberately serve it on a local network.
The portable app has no runtime package installation, account or cloud service.

## Included models

| Model | Geometry / authority |
| --- | --- |
| F28 mean | Mean-based adult female reference; 92 ANSUR II women aged exactly 28 |
| F28 refined | Shoulders −1%, waist −3%, hip +2%; design choices remain explicit |
| SOPHY scale | Proposed form overlay at H=span=1676.4 mm; canon kernel unchanged |
| A0-R1 | Exact current 19 B-rep solids / 35 located instances; separate bench article |
| Source library | Existing 18 source-based envelope solids; gallery placement only |

Orbit, zoom, select and isolate parts; inspect provenance and dimensions; compare
A/T poses and display palettes; inspect exploded views; measure between surfaces;
download GLB, STL or a JSON review state. The supported quarter-scale maquette ZIP
is embedded in the HTML, including cut SVGs, BOM and assembly instructions.

Palette changes affect appearance. They never revise a construction BOM. The form
envelope is interpolated design, not a scan. Couplings are envelopes without
internally defined mechanisms. A0 does not inherit whole-body scales or poses.

Exports include the complete frame (form envelope and guides are separate), current
pose, scale and separation. STL is in the canonical X/Y/Z frame in millimeters;
GLB uses glTF's Y-up frame in meters. JSON includes the original model plus the
review state, preserving the distinction between source and exploration.

## Rebuild

```bash
npm ci --prefix studio
npm run build --prefix studio
python -m pip install cadquery==2.7.0
python bin/export_phy_studio.py
```

Outputs go to `studio/dist/`: offline HTML, three reference JSONs and STLs,
source report, maquette package, manifest hashes and a complete ZIP.
For a standard-library-only body/maquette rebuild:

```bash
python bin/export_phy_studio.py --without-a0
```

The checked-in `app.bundle.js` contains three.js 0.180.0, official OrbitControls
and exporters. Rebuilding viewer source needs Node; running the portable app does
not. The Three MIT notice is included in the app and `THIRD_PARTY_NOTICES.txt`.

## Engineering status

The Studio is a runnable demonstration, and the reference assembly is complete as
a visual form study. Full-scale physical readiness is open. The A0 bearing-impact
screen and supplier/clearance/physical-evidence gates remain visible and unchanged.
The maquette is a supported, passive plywood form study, not a functional armature.
See `PROJECTS/PHY_F28/README.md` and the in-app Build path / Readiness panels.
