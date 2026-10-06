# Core bone proxy layout / CORE_V1

Project-local provisional placement, not osteometry, joint anatomy or canon adoption.
Values are mm at the cohort mean stature unless the key states fraction, degrees or samples.
Mean/refined/SOPHY overlays scale these choices uniformly; all dimensional fidelity is unverified.
Source: `PROJECTS/PHY_F28/profiles/f28.json#/bone_proxy_layout`

Layout SHA-256: `2534b4d17ee1598d28096c493b6a70ce01f51dceffdfe15fc73750f7c690f7d3`

| Input | Governing value |
| --- | --- |
| revision | `"CORE_V1"` |
| authority | `"project-local provisional topology and placement; not osteometric data or canon"` |
| units | `"mm at cohort mean stature; uniformly scaled for overlays"` |
| c1_below_vertex_mm | `175` |
| t1_below_cervicale_mm | `25` |
| l1_above_tenth_rib_mm | `20` |
| t12_above_l1_mm | `30` |
| l5_above_hip_mm | `60` |
| spine_y_mm | `[-18, -32, -50, -26, -22, -35, -45, -55]` |
| vertebra_radii_mm | `{"C": [14, 11, 5], "L": [22, 17, 12], "T": [17, 13, 9]}` |
| sacrum_offset_from_hip_mm | `10` |
| sacrum_radii_mm | `[32, 16, 38]` |
| coccyx_below_hip_mm | `80` |
| coccyx_radii_mm | `[9, 8, 17]` |
| rib_breadth_fractions | `[0.55, 0.65, 0.76, 0.87, 0.96, 1.0, 1.0, 0.96, 0.9, 0.81, 0.65, 0.48]` |
| rib_depth_fractions | `[0.48, 0.56, 0.64, 0.7, 0.74, 0.76, 0.77, 0.75, 0.72, 0.68, 0.57, 0.44]` |
| rib_drop_mm | `[15, 20, 26, 32, 38, 42, 44, 42, 36, 28, 18, 12]` |
| rib_radius_mm | `4.5` |
| rib_posterior_offset_mm | `8` |
| rib_sweep_samples | `25` |
| rib_free_end_angle_deg | `42` |
| sternum_half_width_mm | `9` |
| sternum_radius_mm | `5` |
| scapula_vertices_relative_shoulder_mm | `[[-90, -78, 15], [-8, -60, -25], [-60, -95, -145]]` |
| scapula_thickness_mm | `4` |
| hip_plate_half_thickness_mm | `8` |
| hip_outline_offsets_mm | `[10, 40, 25, 55, 25, 8, 20, 35]` |
| hip_outline_breadth_fractions | `[0.5, 0.46, 0.2]` |
| forearm_separation_mm | `[18, 25]` |
| radius_radii_mm | `[7, 5]` |
| ulna_radii_mm | `[8, 5]` |
| patella_offset_mm | `[0, 18, 3]` |
| patella_radii_mm | `[12, 6, 17]` |

External station inputs remain in the hash-pinned reference projection and `design_choices_mm`.
The retained humerus/femur/tibia/fibula/clavicle proxies are station-based forms, not measured bones.
No clearances, structural sections, joint interfaces or manufacturing release are asserted.
