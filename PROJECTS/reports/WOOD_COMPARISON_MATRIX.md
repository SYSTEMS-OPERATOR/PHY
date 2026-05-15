# WOOD COMPARISON MATRIX

Offline summary generated from committed `PROJECTS/*/materials/*.json` records.

No values are inferred in this report. Blank or missing values remain `null` or are described as unknown.

## Current material records

| Project | Material ID | Common name | Grade | Source status | Main role |
|---|---|---|---|---|---|
| REDWOOD | coast_redwood_average | Coast redwood | standard | verified | primary build candidate |
| REDWOOD | coast_redwood_old_growth | Coast redwood | old_growth | partial | old-growth / reclaimed comparison candidate |
| CYPRESS | bald_cypress_standard | Bald cypress | standard | verified | water-endurance candidate |
| JUNIPER | western_juniper_standard | Western juniper | standard | partial | durable arid-zone candidate |
| DEFAULT | douglas_fir_standard | Douglas fir | standard | verified | structural prototype baseline |
| DEFAULT | hard_maple_standard | Hard maple | standard | verified | dense/hard prototype control |
| DEFAULT | yellow_poplar_standard | Yellow poplar | standard | verified | easy-carve draft stock |
| BRISTLECONE | great_basin_bristlecone_reference | Great Basin bristlecone pine | theoretical | partial | endurance reference only |

## Mechanical property matrix

| Material ID | Density kg/m3 | SG basic | SG 12% MC | Janka lbf | MOR MPa | MOE GPa | Crushing MPa | Radial shrink % | Tangential shrink % | Volumetric shrink % |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| coast_redwood_average | 415 | 0.36 | 0.42 | 450 | 61.7 | 8.41 | 39.2 | 2.4 | 4.7 | 6.9 |
| coast_redwood_old_growth | 450 | null | null | 480 | 69.0 | 9.24 | 42.4 | null | null | null |
| bald_cypress_standard | 515 | 0.42 | 0.51 | 510 | 73.1 | 9.93 | 43.9 | 3.8 | 6.2 | 10.5 |
| western_juniper_standard | 440 | 0.40 | 0.44 | 680 | 61.4 | 4.43 | 32.5 | null | null | 8.0 |
| douglas_fir_standard | 510 | 0.45 | 0.51 | 620 | 86.2 | 12.17 | 47.9 | 4.5 | 7.3 | 11.6 |
| hard_maple_standard | 705 | 0.56 | 0.71 | 1450 | 109.0 | 12.62 | 54.0 | 4.8 | 9.9 | 14.7 |
| yellow_poplar_standard | 455 | 0.40 | 0.46 | 540 | 69.7 | 10.90 | 38.2 | 4.6 | 8.2 | 12.7 |
| great_basin_bristlecone_reference | null | null | null | null | null | null | null | null | null | null |

## Qualitative fabrication matrix

| Material ID | Decay resistance | Workability | Grain / stability notes | Fastener behavior |
|---|---|---|---|---|
| coast_redwood_average | Moderately durable to very durable; old-growth tends to be more durable than younger second-growth | Typically easy to work; figured stock may tear out | Generally straight; exceptional stability noted | unknown |
| coast_redwood_old_growth | Old-growth tends to be more durable than younger second-growth | Typically easy to work; figured stock may tear out | Generally straight; exceptionally stable noted | unknown |
| bald_cypress_standard | Old-growth durable to very durable; younger-tree wood moderately durable | Sharp cutters and light passes recommended to avoid tearout; good gluing, nailing, finishing, and paint-holding | Straight grain, medium to coarse texture; raw surfaces may feel greasy | good nailing properties reported |
| western_juniper_standard | Durable; resistant to decay and termite attack | Generally easy, but knots or grain irregularity may cause difficulty | Usually straight, fine even texture; knots and irregularity possible | unknown |
| douglas_fir_standard | Heartwood moderately durable for decay; susceptible to insect attack | Machines well; moderate blunting effect; accepts stains, glues, and finishes | Generally straight or slightly wavy; medium to coarse texture | unknown |
| hard_maple_standard | Non-durable to perishable; susceptible to insect attack | Fairly easy but harder than soft maple; can burn with high-speed cutters; turns, glues, finishes well | Generally straight or wavy; fine even texture | unknown |
| yellow_poplar_standard | Moderately durable to non-durable; susceptible to insect attack | Very easy to work; low density can leave fuzzy surfaces and edges | Typically straight, uniform grain | unknown |
| great_basin_bristlecone_reference | Not quantified | Not quantified | Not quantified | Not quantified |

## Offline design conclusions

### Current primary build candidate

`coast_redwood_average` remains the strongest practical candidate for a visible armature material because it combines low density, low shrinkage, workable softness, and high stability compared with the DEFAULT baselines.

### Current old-growth comparison candidate

`coast_redwood_old_growth` has higher listed density, hardness, MOR, MOE, and crushing strength than the average coast redwood record, but old-growth-specific SG and shrinkage remain unknown in the current local record.

### Current water-endurance candidate

`bald_cypress_standard` is the strongest verified CYPRESS record and is useful for comparing redwood against a decay-resistant, stable, water-associated wood.

### Current durable-arid candidate

`western_juniper_standard` is useful but incomplete. The record has verified density, SG, hardness, MOR, MOE, crushing strength, and volumetric shrinkage, but radial and tangential shrinkage are still unknown.

### Current prototype controls

`douglas_fir_standard` is the strength/stiffness prototype baseline.

`hard_maple_standard` is the dense/hard prototype-control material.

`yellow_poplar_standard` is the easy-carve draft stock.

### Current bristlecone status

`great_basin_bristlecone_reference` is not a fabrication material record. It is a theoretical endurance reference with no mechanical properties entered. The record explicitly does not authorize fabrication use.

## Missing work

- Add exact source-backed legal/sourcing records for reclaimed or ancient-grade material availability.
- Add component-use scoring only after criteria are written and reviewed.
- Add at least one material record per project for hardware contact assumptions if copper/brass interfaces are compared.
- Add local tests for fastener behavior, drill tearout, bushing retention, and finish behavior.
- Add REDWOOD component guides for a small first bone set: femur, humerus, rib template, phalanx template, scapula, pelvis.

## References by source record

See each JSON record under `PROJECTS/*/materials/` for source URLs and exact unknown tracking.
