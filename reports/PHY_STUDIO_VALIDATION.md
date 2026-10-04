# PHY Studio validation — 2026-10-04

Status: **RUNNABLE DEMONSTRATION AND REPRODUCIBLE FORM-STUDY PACKAGE**.
Full-scale structural fabrication release: **false**. Physical build/bench evidence:
**unmeasured**. Existing SOPHY canon and A0 design sources remain unchanged.

| Check | Result |
| --- | --- |
| Full local repository suite | 138 passed, 5 skipped, 339 subtests passed |
| Skips | Four PyBullet-dependent modules (PyBullet absent); one Docker-dependent test |
| New source/geometry/maquette/A0 suite | 8 passed; 102 subtests passed |
| Source evidence | 92 women aged exactly 28; pinned CSV/subset hash and overlapping-field agreement |
| Whole-body reference variants | Mean, gently refined, SOPHY-scale comparison; 94 meshes each |
| Mesh integrity | Closed edge incidence, nondegenerate triangles, positive signed volume |
| Geometry closure | Exact mirrored joint datums, floor/vertex and declared T-pose span |
| A0 viewer | 19 existing B-rep parts / 35 instances; pre-tessellation bounds match existing register |
| Browser | Headless Chromium, real WebGL with software renderer; zero console/page errors |
| Interaction | Five models, part picking/isolation/focus, two-point surface ruler, height, pose and materials |
| Export | Valid binary STL and GLB containers; JSON review state; complete frame in current scale/pose |
| Export scale | STL at 1700 mm selected height reaches 1700 mm in canonical coordinates; GLB uses meters |
| Maquette download | Embedded ZIP opens with valid CRCs; SVG cut paths, assembly stencil, BOM and instructions |
| Mobile | 390 px viewport; no horizontal page overflow; controls remain reachable |
| Local launcher | HTTP 200 from the standard-library server; portable HTML also tested via file:// |
| Offline minimal rebuild | python -S export with --without-a0 succeeds, without site packages |
| Repeat export | Every output file, including the offline HTML and complete ZIP, byte-identical |

The full suite emits existing warnings about incomplete biological metrics and
deprecated datetime usage. Those are not physical inspection or approval evidence.
The new browser smoke test is `studio/tests/smoke.mjs`; a dedicated GitHub Actions
workflow rebuilds the package, checks geometry and determinism, and exercises the
offline app and its downloads. The general test workflow includes the httpx2
dependency required by the current Starlette TestClient.

## Engineering limits

- ANSUR II is a military reference sample, not a representative civilian/global mean.
- External segment measurements are not osteometric bone lengths or joint centers.
- Joint datums, couplings, curves, head height and interpolated envelope are explicit
  project design proposals; sources are never overwritten by refinements.
- The supported quarter-scale plywood maquette is a separate passive embodiment.
  Its cuts are generated, but no physical cutting or assembly is claimed.
- A0 remains a separate unscaled bench article. Its 3.25 kN impact bearing screen
  exceeds the unverified 3.0 kN threshold; supplier correlation, stop response,
  continuous clearance, independent review and physical bench records remain open.
- Full-scale joint internals, stability/support, interfaces, loads, actuation and
  fabrication inspection are not closed by a visual reference assembly.
