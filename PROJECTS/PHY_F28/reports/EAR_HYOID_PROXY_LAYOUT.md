# Individual ear and hyoid proxy layout / EAR_HYOID_V1

Seven identities: malleus, incus and stapes on each side, plus one midline hyoid.
Together with the preserved 199 proxies, all 206 inventory identities are represented individually.
Count/distribution coverage does not establish measured morphology, working mechanics or fabrication readiness.
Source: `PROJECTS/PHY_F28/profiles/f28.json#/ear_hyoid_proxy_layout`

Layout SHA-256: `c28df92e058ccd8b50672df576eb09c0830dd4c744ccb3eac4401f1ebe5ab6b2`

Origins use the external half head breadth/depth and half the proposed 220 mm head height.
Local centerlines, offsets and sections are provisional millimeter display choices, scaled from the F28 mean height.
Three authored right ossicles reflect vertices, triangle winding, anchors and same-side chain IDs to the left.
Malleus and incus use tapered sweeps; stapes uses a closed stirrup loop with an open center.
The malleus–incus–stapes chain runs lateral to medial within the head display frame.
Temporal housing is a placement label, not an articulation, cavity, fit or clearance proof.
The tympanic membrane and oval window are endpoint labels; their geometry and mechanisms are unresolved.
The hyoid is one open U below the jaw, with posterior ends and no direct bone articulation.
Its muscle/ligament suspension, soft-tissue interfaces, hearing and swallowing mechanics remain unmodeled.

| Input | Governing value |
| --- | --- |
| revision | `"EAR_HYOID_V1"` |
| authority | `"project-local provisional identity and placement; not measured osteometry, hearing/swallowing mechanics or canon"` |
| units | `"origins: fractions of external head half-breadth, half-depth and proposed half-height; local offsets, centerlines and sections: mm at cohort mean stature"` |
| ear_origin_head_fraction | `[0.67, -0.08, -0.25]` |
| hyoid_origin_head_fraction | `[0, 0.3, -1.05]` |
| ossicles | `{"INCUS": {"center_offset_mm": [0, 0, 0], "centerline_mm": [[0, -2, 1], [0.5, -0.5, 1.3], [0.3, 1, 1.2], [-1, 1.3, 0], [-2, 1.5, -2.5]], "closed_centerline": false, "radii_mm": [0.55, 1.1, 1.0, 0.65, 0.4]}, "MALLEUS": {"center_offset_mm": [5, 0, 0], "centerline_mm": [[0, 0, 3], [0, 0, 1], [0, 0.1, -1.5], [-0.8, 0.4, -4]], "closed_centerline": false, "radii_mm": [1.0, 1.3, 0.6, 0.45]}, "STAPES": {"center_offset_mm": [-5, 0, 0], "centerline_mm": [[0, 0, 1.8], [0, 0.8, 1.4], [0, 1.25, 0.4], [0, 1.3, -0.8], [0, 0.6, -1.4], [0, -0.6, -1.4], [0, -1.3, -0.8], [0, -1.25, 0.4], [0, -0.8, 1.4]], "closed_centerline": true, "radii_mm": 0.3}}` |
| hyoid | `{"centerline_mm": [[18, -10, 0], [18, 0, -1], [12, 7, -2], [0, 10, -2], [-12, 7, -2], [-18, 0, -1], [-18, -10, 0]], "closed_centerline": false, "radii_mm": 2.2}` |

Identity and selected topology references (not numerical geometry sources):
https://openstax.org/books/anatomy-and-physiology-2e/pages/14-1-sensory-perception
https://openstax.org/books/anatomy-and-physiology-2e/pages/7-2-the-skull

All seven identities have existing canonical source modules; their legacy dimension dictionaries remain unchanged.
Those records do not qualify these display meshes. All 193 source records are preserved; 13 facial records remain missing.
CORE_V1, HAND_V1, FOOT_V1 and SKULL_V1 geometry and inputs are preserved.
Dimensional fidelity unverified; physical evidence unmeasured; fabrication release false.
