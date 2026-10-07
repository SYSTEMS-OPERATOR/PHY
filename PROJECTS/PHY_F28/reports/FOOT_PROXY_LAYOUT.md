# Individual foot proxy layout / FOOT_V1

26 identities per foot: seven tarsals, five metatarsals and 14 toe phalanges.
Static grounded display; measured surfaces, arches, ankle/toe mechanics and load paths remain unresolved.
Numeric values are project-local provisional display choices, not osteometry or canon.
Source: `PROJECTS/PHY_F28/profiles/f28.json#/foot_proxy_layout`

Layout SHA-256: `a56fdc7af929de3574974e7b4ecacbd90cc3dd7d7d1607c6c9ba1672a71fe5ff`

Right frame: +x lateral, +y anterior, +z above the floor, heel located behind the ankle station.
Hallux is medial. Left geometry, winding, anchors and topology IDs reflect authored right geometry.

| Input | Governing value |
| --- | --- |
| revision | `"FOOT_V1"` |
| authority | `"project-local provisional display layout; not osteometric data or canon"` |
| units | `"centers/stations: lateral breadth fraction, anterior heel-to-toe fraction, height above floor mm; tarsal radii: breadth fraction, length fraction, vertical mm; digit radii/gaps: mm at cohort mean stature"` |
| heel_behind_ankle_fraction | `0.22` |
| joint_gap_mm | `1.5` |
| tarsals | `{"CALCANEUS": {"center": [0.04, 0.13, 18], "radii": [0.17, 0.13, 18]}, "CUBOID": {"center": [0.28, 0.39, 24], "radii": [0.17, 0.065, 10]}, "INTERMEDIATE_CUNEIFORM": {"center": [-0.075, 0.46, 33], "radii": [0.09, 0.045, 9]}, "LATERAL_CUNEIFORM": {"center": [0.12, 0.46, 29], "radii": [0.095, 0.048, 9]}, "MEDIAL_CUNEIFORM": {"center": [-0.31, 0.46, 30], "radii": [0.13, 0.06, 10]}, "NAVICULAR": {"center": [-0.16, 0.35, 40], "radii": [0.18, 0.045, 9]}, "TALUS": {"center": [0, 0.22, 51], "radii": [0.17, 0.09, 12]}}` |
| digits | `{"1": {"radii_mm": [5.5, 4.3, 3.5], "root": "MEDIAL_CUNEIFORM", "stations": [[-0.32, 0.52, 30], [-0.35, 0.74, 14], [-0.36, 0.89, 11], [-0.36, 0.98, 9]]}, "2": {"radii_mm": [4.3, 3.1, 2.5, 2], "root": "INTERMEDIATE_CUNEIFORM", "stations": [[-0.07, 0.49, 36], [-0.11, 0.78, 14], [-0.13, 0.88, 12], [-0.13, 0.95, 10], [-0.13, 1, 9]]}, "3": {"radii_mm": [4.1, 2.9, 2.3, 1.9], "root": "LATERAL_CUNEIFORM", "stations": [[0.13, 0.52, 31], [0.1, 0.77, 14], [0.08, 0.86, 12], [0.08, 0.92, 10], [0.08, 0.96, 9]]}, "4": {"radii_mm": [3.8, 2.7, 2.2, 1.8], "root": "CUBOID", "stations": [[0.29, 0.49, 24], [0.3, 0.74, 13], [0.3, 0.82, 11], [0.3, 0.88, 10], [0.3, 0.91, 9]]}, "5": {"radii_mm": [3.5, 2.5, 2, 1.6], "root": "CUBOID", "stations": [[0.43, 0.44, 20], [0.44, 0.69, 12], [0.44, 0.76, 10], [0.44, 0.815, 9], [0.44, 0.85, 8]]}}` |

External heel-to-toe length and breadth size a display envelope, not individual bone measurements.
Sections/elevations/gaps scale uniformly; tarsal horizontal sections use the external breadth/length.
Heel and nominal second-toe terminal station span the input length; tube ends are trimmed for display gaps.
This nominal station closure is not exact mesh-envelope conformity or anatomical endpoint adoption.
Selected chain parents describe partial topology, not a complete joint/contact graph.
Gaps are not qualified fits; no continuous clearance, gait, arch function or structural proof is asserted.
Dimensional fidelity unverified; physical evidence unmeasured; fabrication release false.
