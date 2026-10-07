# Individual skull proxy layout / SKULL_V1

22 identities: eight cranial and 14 facial bones, including one adult mandible.
Closed shell patches, ellipsoids and open-centerline sweeps are provisional display geometry.
The shell patches have capped edges. No sutures, foramina, sinuses, teeth or articular surfaces are reproduced.
Source: `PROJECTS/PHY_F28/profiles/f28.json#/skull_proxy_layout`

Layout SHA-256: `ad6a9ccff22eda308e6262af399217596d5ef67d6a874d2678928b8895115ddc`

Fractions use external half head breadth/depth and half of the proposed 220 mm head height.
ANSUR head length is anterior/posterior depth, not crown-to-chin height or a bone dimension.
Six midline and eight right-side proxies are authored; eight left counterparts reflect vertices, winding and selected adjacency IDs.
The separate mandible has bilateral temporal placement stations; no joint axis, fit or motion is implemented.

| Input | Governing value |
| --- | --- |
| revision | `"SKULL_V1"` |
| authority | `"project-local provisional identity and display layout; not osteometric data or canon"` |
| units | `"centers/radii/centerlines: fractions of external head half-breadth, half-depth and proposed half-height; shell inset/thickness and sweep radius: mm at cohort mean stature; shell angles: degrees"` |
| shell_inset_mm | `3` |
| shell_thickness_mm | `2.5` |
| shell_samples | `[12, 8]` |
| shells | `{"FRONTAL": {"azimuth_deg": [40, 140], "polar_deg": [40, 80]}, "OCCIPITAL": {"azimuth_deg": [220, 320], "polar_deg": [42, 105]}, "PAR_R": {"azimuth_deg": [-35, 35], "polar_deg": [14, 78]}, "TEMP_R": {"azimuth_deg": [-38, 28], "polar_deg": [83, 116]}}` |
| ellipsoids | `{"ETHMOID": {"center": [0, 0.36, -0.2], "radii": [0.07, 0.13, 0.21]}, "LACRIMAL_R": {"center": [0.21, 0.72, -0.2], "radii": [0.03, 0.055, 0.1]}, "MAXILLA_R": {"center": [0.25, 0.72, -0.58], "radii": [0.24, 0.12, 0.19]}, "NASAL_R": {"center": [0.058, 0.86, -0.12], "radii": [0.04, 0.06, 0.14]}, "PALATINE_R": {"center": [0.2, 0.31, -0.77], "radii": [0.19, 0.17, 0.03]}, "SPHENOID": {"center": [0, -0.1, -0.31], "radii": [0.66, 0.17, 0.09]}, "VOMER": {"center": [0, 0.39, -0.58], "radii": [0.025, 0.21, 0.15]}}` |
| sweeps | `{"INFERIOR_NASAL_CONCHA_R": {"centerline": [[0.13, 0.66, -0.43], [0.21, 0.6, -0.48], [0.24, 0.46, -0.49], [0.21, 0.34, -0.46]], "radius_mm": 2}, "MANDIBLE": {"centerline": [[0.79, -0.1, -0.33], [0.79, -0.1, -0.57], [0.74, -0.04, -0.8], [0.66, 0.3, -0.85], [0.49, 0.6, -0.86], [0.27, 0.8, -0.86], [0, 0.86, -0.86], [-0.27, 0.8, -0.86], [-0.49, 0.6, -0.86], [-0.66, 0.3, -0.85], [-0.74, -0.04, -0.8], [-0.79, -0.1, -0.57], [-0.79, -0.1, -0.33]], "radius_mm": 6}, "ZYGOMATIC_R": {"centerline": [[0.82, -0.03, -0.32], [0.9, 0.25, -0.34], [0.65, 0.57, -0.34], [0.53, 0.74, -0.36]], "radius_mm": 4.5}}` |

Selected adjacency and identity reference (not a measured morphology source):
https://openstax.org/books/anatomy-and-physiology-2e/pages/7-2-the-skull

Selected adjacency is not a complete suture/articulation graph or a mesh contact/clearance proof.
Nine skull identities have legacy source records; 13 facial records remain explicitly missing.
All 193 canonical source records are unchanged. Their existing dimensions do not qualify these proxies.
The five retained head arches/bands are form-study meshes with no bone correspondence.
Dimensional fidelity unverified; physical evidence unmeasured; fabrication release false.
