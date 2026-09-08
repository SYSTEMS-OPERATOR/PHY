# T-5.6 reference dimension comparison

**Reference candidates only; not fabrication release or canon adoption.**

H = 1676.4 mm; matched sample n = 80 of 1986.

Measured values are unscaled. Legacy values alone are scaled from REDWOOD for comparison. Endpoint mismatches remain explicit.

| Measure | Median mm | Observed p05–p95 mm | Scaled legacy mm | Delta mm | Mapping |
| --- | ---: | ---: | ---: | ---: | --- |
| [acromionradialelength](https://tools.openlab.psu.edu/publicData/ANSURII-TR15-007.pdf#page=59) | 317 | 306.9–332 | 314.96 | 2.04 | external_segment_proxy_not_bone_or_joint_center_length |
| [biacromialbreadth](https://tools.openlab.psu.edu/publicData/ANSURII-TR15-007.pdf#page=69) | 369.5 | 340–390.1 | 375.92 | -6.42 | same_named_measure_requires_endpoint_confirmation |
| [bicristalbreadth](https://tools.openlab.psu.edu/publicData/ANSURII-TR15-007.pdf#page=73) | 281 | 243.9–313.1 | 289.56 | -8.56 | pelvis_breadth_definition_unresolved_not_hip_center_spacing |
| [chestbreadth](https://tools.openlab.psu.edu/publicData/ANSURII-TR15-007.pdf#page=99) | 271 | 245.95–299.05 | 284.48 | -13.48 | external_envelope_proxy_not_structural_housing |
| [chestdepth](https://tools.openlab.psu.edu/publicData/ANSURII-TR15-007.pdf#page=103) | 248 | 207.95–291.1 | — | — | external_envelope_proxy_not_structural_housing |
| [footlength](https://tools.openlab.psu.edu/publicData/ANSURII-TR15-007.pdf#page=125) | 250 | 235.9–262.25 | 243.84 | 6.16 | same_named_measure_requires_endpoint_confirmation |
| [handlength](https://tools.openlab.psu.edu/publicData/ANSURII-TR15-007.pdf#page=141) | 183 | 173.95–190.05 | 177.8 | 5.2 | same_named_measure_requires_endpoint_confirmation |
| [radialestylionlength](https://tools.openlab.psu.edu/publicData/ANSURII-TR15-007.pdf#page=187) | 244.5 | 232–260 | 243.84 | 0.66 | external_segment_proxy_not_bone_or_joint_center_length |
| [span](https://tools.openlab.psu.edu/publicData/ANSURII-TR15-007.pdf#page=201) | 1680 | 1658.9–1700 | — | — | canon_span_comparison |
| [stature](https://tools.openlab.psu.edu/publicData/ANSURII-TR15-007.pdf#page=203) | 1672 | 1653–1694 | — | — | canon_scale_comparison |

The percentile columns describe this selected sample; they are not uncertainty bounds or manufacturing tolerances. Marginal medians do not constitute one observed body. The JSON includes an actual representative row and its nonzero span residual; no body is morphed to fit H.

## Historical comparison

- [footlength](https://www.gutenberg.org/files/20239/20239-h/20239-h.htm#Page_72): historical 279.40 mm versus measured-cohort median 250.00 mm. Neither is silently adopted.
- [handlength](https://www.gutenberg.org/files/20239/20239-h/20239-h.htm#Page_72): historical 167.64 mm versus measured-cohort median 183.00 mm. Neither is silently adopted.

## Selection sensitivity

- ±10 mm on both stature and span: n=12; sufficient=False.
- ±25 mm on both stature and span: n=80; sufficient=True.
- ±50 mm on both stature and span: n=253; sufficient=True.

## Supporting references

- [NASA-STD-3001 Volume 2, Appendix E: Physical Characteristics and Capabilities Data Sets](https://www.nasa.gov/reference/appendix-e-vol-2/), sections E.2, E.3, E.5: landmark_and_motion_review_guidance_no_numeric_import.

## Still unresolved

- head_height_mm: ANSUR headlength is front-to-back, not chin-to-crown height; select a matched endpoint source.
- humerus_length_mm: Acromion-radiale is a proxy; osteometric endpoints and shoulder/elbow centers require separate definition.
- radius_length_mm: Radiale-stylion is a proxy; bone ends and cartridge datums require separate definition.
- ulna_length_mm: Do not copy radius or forearm length into ulna length.
- femur_length_mm: Standing external heights are not femoral head-to-condyle length.
- tibia_length_mm: Tibiale height is not isolated tibia length or knee-to-ankle center spacing.
- fibula_length_mm: Do not derive from tibia length without a selected endpoint model.
- joint_centers_and_mechanism_paths: Scalar anthropometry cannot determine 3D centers, shoulder trajectories, bearings, stops, or fixture interfaces.
