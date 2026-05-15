# Appendix A — Material Specification Appendix

## A.1 Purpose

This appendix summarizes the current wood and building-material records available under `PROJECTS/*/materials/`.

It is not a substitute for the machine-readable JSON records. It is a human-readable overview for offline design review.

No value should be added here unless it exists in a committed material record or is clearly marked as unknown.

## A.2 Current material families

```text
REDWOOD
  coast_redwood_average
  coast_redwood_old_growth

CYPRESS
  bald_cypress_standard

JUNIPER
  western_juniper_standard

BRISTLECONE
  great_basin_bristlecone_reference

DEFAULT
  douglas_fir_standard
  hard_maple_standard
  yellow_poplar_standard
```

## A.3 Coast redwood — average

Material ID: `coast_redwood_average`

Project: REDWOOD

Role: primary visible build candidate.

Known values in local record:

- density: 415 kg/m3
- basic SG: 0.36
- SG at 12 percent MC: 0.42
- Janka: 450 lbf
- MOR: 61.7 MPa
- MOE: 8.41 GPa
- crushing strength: 39.2 MPa
- radial shrinkage: 2.4 percent
- tangential shrinkage: 4.7 percent
- volumetric shrinkage: 6.9 percent

Design reading:

Low density and low shrinkage make it a strong visible candidate for carved armature parts where stability and workability matter more than maximum hardness.

Key unknowns:

- fastener behavior
- component-use scores
- ancient-grade availability

## A.4 Coast redwood — old growth

Material ID: `coast_redwood_old_growth`

Project: REDWOOD

Role: old-growth / reclaimed comparison candidate.

Known values in local record:

- density: 450 kg/m3
- Janka: 480 lbf
- MOR: 69.0 MPa
- MOE: 9.24 GPa
- crushing strength: 42.4 MPa

Unknown in local record:

- old-growth-specific basic SG
- old-growth-specific SG at 12 percent MC
- old-growth-specific radial shrinkage
- old-growth-specific tangential shrinkage
- old-growth-specific volumetric shrinkage

Design reading:

Old-growth redwood is promising, but it cannot be assumed superior in every refined-aging behavior until sample movement and finish tests are performed.

## A.5 Bald cypress — standard

Material ID: `bald_cypress_standard`

Project: CYPRESS

Role: water-endurance and decay-resistance comparator.

Known values in local record:

- density: 515 kg/m3
- basic SG: 0.42
- SG at 12 percent MC: 0.51
- Janka: 510 lbf
- MOR: 73.1 MPa
- MOE: 9.93 GPa
- crushing strength: 43.9 MPa
- radial shrinkage: 3.8 percent
- tangential shrinkage: 6.2 percent
- volumetric shrinkage: 10.5 percent

Design reading:

Bald cypress is a strong endurance challenger where decay resistance and damp-risk behavior matter.

Key unknowns:

- component-use scores
- sinker/ancient-grade mechanical differences

## A.6 Western juniper — standard

Material ID: `western_juniper_standard`

Project: JUNIPER

Role: arid durability and dense detail candidate.

Known values in local record:

- density: 440 kg/m3
- basic SG: 0.40
- SG at 12 percent MC: 0.44
- Janka: 680 lbf
- MOR: 61.4 MPa
- MOE: 4.43 GPa
- crushing strength: 32.5 MPa
- volumetric shrinkage: 8.0 percent

Unknown in local record:

- radial shrinkage
- tangential shrinkage
- fastener behavior
- component-use scores

Design reading:

Western juniper is harder than redwood and cypress in the current matrix and may be useful for small components, inserts, or joint blocks, but grain irregularity and incomplete shrinkage data require coupon testing.

## A.7 Douglas fir — standard

Material ID: `douglas_fir_standard`

Project: DEFAULT

Role: strength/stiffness prototype baseline.

Known values in local record:

- density: 510 kg/m3
- basic SG: 0.45
- SG at 12 percent MC: 0.51
- Janka: 620 lbf
- MOR: 86.2 MPa
- MOE: 12.17 GPa
- crushing strength: 47.9 MPa
- radial shrinkage: 4.5 percent
- tangential shrinkage: 7.3 percent
- volumetric shrinkage: 11.6 percent

Design reading:

Douglas fir is useful for prototype structural members and stiffness comparison. It is not the final visual material by default.

## A.8 Hard maple — standard

Material ID: `hard_maple_standard`

Project: DEFAULT

Role: dense/hard prototype-control material.

Known values in local record:

- density: 705 kg/m3
- basic SG: 0.56
- SG at 12 percent MC: 0.71
- Janka: 1450 lbf
- MOR: 109.0 MPa
- MOE: 12.62 GPa
- crushing strength: 54.0 MPa
- radial shrinkage: 4.8 percent
- tangential shrinkage: 9.9 percent
- volumetric shrinkage: 14.7 percent

Design reading:

Hard maple is a high-hardness control and possible wear-test material, but it is not decay-endurance stock in the current local record.

## A.9 Yellow poplar — standard

Material ID: `yellow_poplar_standard`

Project: DEFAULT

Role: easy-carve draft stock.

Known values in local record:

- density: 455 kg/m3
- basic SG: 0.40
- SG at 12 percent MC: 0.46
- Janka: 540 lbf
- MOR: 69.7 MPa
- MOE: 10.90 GPa
- crushing strength: 38.2 MPa
- radial shrinkage: 4.6 percent
- tangential shrinkage: 8.2 percent
- volumetric shrinkage: 12.7 percent

Design reading:

Yellow poplar is a good sacrificial prototype and shape-study stock. It should not be treated as a final endurance recommendation without further tests.

## A.10 Great Basin bristlecone pine — reference

Material ID: `great_basin_bristlecone_reference`

Project: BRISTLECONE

Role: age/endurance reference only.

Known values in local record:

- wood density: 26.83 lb/ft3
- converted density: 429.8 kg/m3
- FIA specific gravity: 0.43

Unknown in local record:

- Janka hardness
- MOR
- MOE
- crushing strength
- radial shrinkage
- tangential shrinkage
- volumetric shrinkage
- 12 percent moisture-content engineering table

Design reading:

Bristlecone supports the age/endurance hypothesis as a biological reference. It is not a fabrication material specification and does not authorize cut-stock use.

## A.11 Summary ranking by current role

| Role | Current best candidate |
|---|---|
| final visible primary stock | coast redwood average |
| old-growth comparison | coast redwood old growth |
| damp/decay endurance challenger | bald cypress standard |
| dense small-part challenger | western juniper standard |
| strength/stiffness prototype baseline | Douglas fir standard |
| hard wear-control prototype | hard maple standard |
| low-cost shape prototype | yellow poplar standard |
| age/endurance philosophy | Great Basin bristlecone reference |

## A.12 Appendix maintenance rule

When a material JSON record changes, this appendix must be reviewed.

The JSON record is the source of truth. This appendix is the human-readable digest.
