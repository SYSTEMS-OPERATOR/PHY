# Individual hand proxy layout / HAND_V1

27 identities per hand: eight carpals, five metacarpals and 14 phalanges.
Static radial/palmar thumb review pose; working opposition and all joint interfaces remain unresolved.
Numeric values are project-local provisional display choices, not measured bone lengths or canon.
Source: `PROJECTS/PHY_F28/profiles/f28.json#/hand_proxy_layout`

Layout SHA-256: `06d9d47ec5d917171778f4d60e39e51ab720ae7c2f660363def7985312253148`

The wrist-relative frame follows the distal arm, radial radius side and anterior palmar direction.
Left hand geometry, winding, anchors and topology IDs reflect the authored right side.

| Input | Governing value |
| --- | --- |
| revision | `"HAND_V1"` |
| authority | `"project-local provisional display layout; not osteometric data or canon"` |
| units | `"centers/stations: distal hand-station fraction, radial hand-breadth fraction, palmar mm; sections/gaps: mm at cohort mean stature"` |
| joint_gap_mm | `2` |
| carpals | `{"CAPITATE": {"center": [0.2, -0.07, 0], "radii_mm": [8, 6, 6], "row": "distal"}, "HAMATE": {"center": [0.2, -0.26, 0], "radii_mm": [8, 7, 5], "row": "distal"}, "LUNATE": {"center": [0.08, 0, 0], "radii_mm": [7, 6, 5], "row": "proximal"}, "PISIFORM": {"center": [0.08, -0.24, 12], "radii_mm": [4.5, 4.5, 4.5], "row": "proximal"}, "SCAPHOID": {"center": [0.08, 0.18, 0], "radii_mm": [9, 6, 5], "row": "proximal"}, "TRAPEZIUM": {"center": [0.2, 0.3, 0], "radii_mm": [7, 6, 5], "row": "distal"}, "TRAPEZOID": {"center": [0.2, 0.12, 0], "radii_mm": [6, 5, 5], "row": "distal"}, "TRIQUETRUM": {"center": [0.08, -0.24, 0], "radii_mm": [7, 6, 5], "row": "proximal"}}` |
| digits | `{"1": {"radii_mm": [4, 3.3, 2.5], "root": "TRAPEZIUM", "stations": [[0.26, 0.34, 6], [0.48, 0.6, 12], [0.64, 0.78, 16], [0.76, 0.86, 18]]}, "2": {"radii_mm": [4.2, 3.4, 2.8, 2.2], "root": "TRAPEZOID", "stations": [[0.26, 0.24, 0], [0.57, 0.26, 0], [0.76, 0.26, 0], [0.88, 0.26, 0], [0.95, 0.26, 0]]}, "3": {"radii_mm": [4.3, 3.6, 2.9, 2.3], "root": "CAPITATE", "stations": [[0.26, 0, 0], [0.57, 0, 0], [0.78, 0, 0], [0.91, 0, 0], [1, 0, 0]]}, "4": {"radii_mm": [4, 3.4, 2.8, 2.2], "root": "HAMATE", "stations": [[0.26, -0.23, 0], [0.55, -0.23, 0], [0.75, -0.23, 0], [0.87, -0.23, 0], [0.94, -0.23, 0]]}, "5": {"radii_mm": [3.5, 3, 2.4, 2], "root": "HAMATE", "stations": [[0.26, -0.43, 0], [0.5, -0.43, 0], [0.66, -0.43, 0], [0.75, -0.43, 0], [0.82, -0.43, 0]]}}` |

External hand breadth and span-derived hand station size this display layout only.
Intentional gaps separate display segments; they are not qualified joint fits or clearances.
Carpal row order and selected chain parents describe partial topology, not a complete articulation graph.
Dimensional fidelity unverified; physical evidence unmeasured; fabrication release false.
