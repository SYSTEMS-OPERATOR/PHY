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
| NIH female pelvis | Pinned 19,983-triangle native reference preview; 14 tissue meshes / four fused adult bones |
| NIH female skin | Pinned 25,000-triangle exterior preview; no bone or tissue-mechanics evidence |

Orbit, zoom, select and isolate parts; inspect provenance and dimensions; compare
A/T poses and display palettes; inspect exploded views; measure between surfaces;
download GLB, STL or a JSON review state. The supported quarter-scale maquette ZIP
is embedded in the HTML, including cut SVGs, BOM and assembly instructions.

Palette changes affect appearance. They never revise a construction BOM. The form
envelope is interpolated design, not a scan. Couplings are envelopes without
internally defined mechanisms. A0 does not inherit whole-body scales or poses.

## Bone-equivalence inspection

The reference body has **206 individual bone proxies**: the 71 core bones (26 spine, 25 thorax,
four shoulder-girdle, two adult hip, six upper-limb and eight lower-limb) plus
54 hand bones, 52 foot bones, 22 skull bones, six ear ossicles and one hyoid.
Each hand has eight carpals, five metacarpals and 14 phalanges,
with two carpal rows and a radial, palmar-offset static thumb chain.
Each foot has seven tarsals, five metatarsals and 14 toe phalanges, including two
in the medial hallux. Heel/talus and cuneiform/cuboid relationships are inspectable
in a static grounded pose; arches, articular surfaces and gait remain unresolved.
The skull has eight cranial and 14 facial identities, including a separate adult
mandible and mirrored facial pairs. Sutures, foramina, sinuses, teeth and jaw
mechanics are unresolved. Head form guides carry no bone correspondence.
The ears have separate mirrored malleus–incus–stapes chains, including open-center
stapes loops. The midline hyoid is an open U below the jaw, without direct bone
articulations. Their locations and shapes are provisional; soft-tissue suspension,
hearing and swallowing mechanics remain unmodeled. All 206 adult inventory
identities have individual proxies; no grouped or unrepresented identities remain.
All 206 canonical source records are present. The earlier 193 records are preserved;
the 13 facial identity additions retain unknown physical dimensions and interfaces.

Use **Individual bone proxies only** to hide hardware, support rails, head form guides
and envelopes. Search by `BONE_` ID or inspect the Bone coverage panel. Coupling
visibility uses role metadata rather than a brass-material heuristic. The bone-only
filter also governs STL/GLB export; GLB carries bone IDs and role/provenance extras.
Skull and ear/hyoid GLB extras include their layout hashes, selected adjacency and
explicit canonical source-record status; the inspector shows explicit canonical source-record status. Ear extras distinguish temporal housing and non-bone endpoints;
the hyoid identifies its unmodeled muscle/ligament support.
JSON retains the complete original model/audit plus the active review/filter state.
Other display-layer and isolate settings do not limit the exported complete frame.

This closes a bounded count/distribution gap, **not dimensional fidelity**. Shapes,
bone endpoints, joints and morphology are unverified project proposals. None become
BoneSpec dimensions or fabrication-approved interfaces. The static thumb layout
does not establish working opposition; foot placement does not establish gait or
anatomical contact. Skull display geometry does not establish jaw motion.
Left core/hand/foot/skull/ear proxies derive
by reflection of authored right meshes, with triangle winding corrected.

Exports include the complete frame (form envelope and guides are separate), current
pose, scale and separation. STL is in the canonical X/Y/Z frame in millimeters;
GLB uses glTF's Y-up frame in meters. JSON includes the original model plus the
review state, preserving the distinction between source and exploration.

## Engineering evidence and NASA references

**Engineering** opens an offline audit of all 206 canonical identities. The current
review baseline has 40 complete declared dimension maps, 28 improved partial maps
and 128 traced additions from PHY-DIM-072. A complete declaration does not qualify
anatomical geometry, working joints or fabrication. Record validation remains
0/206; physical evidence is unmeasured and release is false.

The component inspector separates the current display envelope from canonical
millimeter declarations. Expand a traced field for its native mean and published
SD, source revision, cohort/side, landmark definition, derivation, snapshot hashes,
rights and limitations. Cohort SD is not a measurement-error bound or a machining
tolerance. Older declarations without these field protocols remain explicitly
identified; missing measurements stay unknown. Changing display height or pose
never changes the evidence values.

The **NASA** tab retains 52 printed motion pairs: 20 unsuited, 16 suited
unpressurized and 16 suited pressurized, plus 11 external anthropometric
comparisons. It retains all 17 source issues and original signs. Condition changes
only select reference rows; they do not change Studio poses or mechanical limits.
HIDH, OCHMO-HB-004 and Appendix E remain comparison/background sources. Human
microgravity stature changes do not rescale the armature. Anatomical landmark
axes and principal inertial axes require separate frames.

`studio/evidence_baseline.json` pins the reviewed engineering revision, canonical
records/source files, proxy snapshot and NASA/dimension catalogs. The exporter
recomputes the existing engineering and dimension audits from fresh source records
before writing evidence; it refuses a stale baseline or failed evidence custody.
After an engineering change, explicitly review and repin the baseline instead of
hand-editing a generated count. Cached audit reports are not inputs to Studio.

The portable app and review JSON carry the evidence; the package also includes
`ENGINEERING_EVIDENCE.json`, `NASA_POSITIONING_RANGES.json` and their manifest hashes.
The footer shows the engineering baseline and package input fingerprint. The
baseline revision names reviewed engineering data; source/bundle hashes identify
this viewer build without timestamps or a self-referential Git HEAD. Source links
open only when selected; evidence inspection needs no network or PDF acquisition.

## Native NIH comparison and draft landmarks

The checked-in viewer includes the two existing small previews from PHY-REF
revision `0cafc8e7f387d009a666b3b3b44bfae29904c53c` (review packet proposed in
[PHY-REF PR 2](https://github.com/SYSTEMS-OPERATOR/PHY-REF/pull/2)).
`studio/reference_lock.json` pins all consumed files, bytes and SHA-256 hashes.
Original binary custody and derived-mesh authority remain in PHY-REF. The HTML
and distribution package embed copies of these exact derived bytes for offline
use, with attribution, CC BY 4.0 license and the existing decimation recipe.

Select **NIH female pelvis** or **NIH female skin**. Coordinates remain source
world Y-up/meters; the renderer applies only 1000 mm/m display conversion. No
centering, mirroring, anatomical-axis mapping, stature normalization or PHY
registration occurs. Views are named by native axes. **View SOPHY separately**
switches frames rather than overlaying unregistered geometry. Camera fits do not
alter the model. Height, pose, palette, explosion and proxy filters are disabled
for source fidelity. Source bounds/ruler distances remain unqualified preview
geometry. The skin supplies no bone dimensions, elasticity, mass or mechanics.

The **NIH references** tab records originals, preview hashes, rights, provenance,
fidelity limits and a pending sacrum–left hemipelvis packet. The pelvis inspector
provides five candidate feature definitions and explicit preview-point capture.
Draft annotations retain mesh/source-node labels, preview node/face/vertices,
barycentric weights and native coordinates. Original-face correspondence and
anatomical acceptance remain null/false. Download the JSON before closing;
points persist only while this page is open. It does not change canonical fields.

Validate the draft in the pinned PHY-REF checkout:

```bash
python tools/validate_landmark_review.py --draft /path/to/PHY-pelvis-preview-draft.json
```

This reconstructs preview geometry only. Original-surface annotation by two
independent reviewers, metric calibration, segmentation review and approved
uncertainty/residual budgets remain required. Accepted points and transforms
stay unknown. A0 bearing/stop blockers remain separate. The dimension baseline
remains 40/206 complete declared maps and SOPHY 1.0.0 remains 1676.4 mm height/span.

Native **GLB** download preserves the pinned preview bytes and meter/Y-up frame;
**JSON** exports provenance and the unaccepted draft. Native STL/reinterpreted
mesh export is unavailable. Existing PHY armature GLB/STL/JSON exports remain.


## Rebuild

```bash
npm ci --prefix studio
npm run build --prefix studio
python -m pip install cadquery==2.7.0
python bin/export_phy_studio.py --reference-root /path/to/pinned-PHY-REF --sync-review
npm run test:logic --prefix studio
```

For an explicit small-preview rebuild, materialize the pinned PHY-REF checkout
with LFS smudge disabled. The exporter checks the lock hashes before publishing;
it reads only the two derived GLBs (824,908 bytes total), metadata, license and
inspector. It performs no acquisition and does not open any original GLB. A
metadata/hash mismatch blocks the build. CI checks out that immutable revision
with LFS disabled. Omitting `--reference-root` builds the existing five-model
configuration (three with `--without-a0`). The native metadata/packet remains
proposed review evidence, not an accepted engineering baseline.

Outputs go to `studio/dist/`: offline HTML, three reference JSONs and STLs,
source report, maquette package, manifest hashes and a complete ZIP.
For a standard-library-only body/maquette rebuild:

```bash
python -S bin/export_phy_studio.py --without-a0 --reference-root /path/to/pinned-PHY-REF
```

The checked-in `app.bundle.js` contains three.js 0.180.0, official OrbitControls
GLTFLoader and exporters. Rebuilding viewer source needs Node; running the portable app does
not. The Three MIT notice is included in the app and `THIRD_PARTY_NOTICES.txt`.

## Engineering status

The Studio is a runnable demonstration, and the reference assembly is complete as
a visual form study. Full-scale physical readiness is open. The A0 bearing-impact
screen and supplier/clearance/physical-evidence gates remain visible and unchanged.
The maquette is a supported, passive plywood form study, not a functional armature
or a miniature of the revised bone distribution. Its earlier 26-cut geometry is
preserved exactly and has its own former/member schedule.
See `PROJECTS/PHY_F28/README.md` and the in-app Build path / Readiness panels.
