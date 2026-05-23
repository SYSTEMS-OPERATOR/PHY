# BONE_VERTEBRA — Vertebra (midline)

## Purpose

Fabrication guide for midline Vertebra as a REDWOOD reference.  
Auto-generated from project anthropometric and anatomical datasets.

## Finished target envelope

- Finished length: `N/A mm` *(see: adult_female_21_28.json)*
- Finished width: `N/A mm`
- Finished thickness: `N/A mm`
- Target side: `midline`
- Bone class: `irregular_bone`

## Biological reference (21-year-old female, avg)

- Biological length: `2.0 cm`
- Biological width: `3.5 cm`
- Biological thickness: `2.5 cm`
- Mass: `10 g`
- Density: `1000 kg/m³`

## Rough Redwood blank

- Blank length: ` mm`
- Blank width: ` mm`
- Blank thickness: ` mm`
- Grain axis: `proximal_to_distal`
- Allowance notes: `10 mm length, 4 mm width/thickness added`

## Carving envelope

- Preserve joint regions until final drilling.
- Maintain minimum dimension at joint ends.
- Suggested centerline: follow grain.

## Joint centers

| End      | Joint center                | Hardware group | Drill target           | Notes                          |
|----------|-----------------------------|---------------|------------------------|---------------------------------|
| Proximal | TBD (see parent module)     | large_primary | 12 mm bushing OD      | To be set by joint module      |
| Distal   | TBD (see child module)      | large_primary | 12 mm bushing OD      | Align with adjacent bone guide |

## Hardware

- Pins: `COPPER_PIN_8MM`
- Bushings: `BRASS_BUSHING_8MM_ID_12MM_OD`
- Washers/spacers: `BRASS_WASHER_8MM`
- Hinge/plate: `BRASS_SINGLE_AXIS_HINGE_LARGE` (if articulated)

## Drill schedule

| Operation                  | Diameter     | Depth    | Reference face        | Notes                           |
|----------------------------|-------------|----------|----------------------|----------------------------------|
| BONE_VERTEBRA_HIP_BUSHING_PILOT   | 12 mm      | TBD      | proximal end         | Validate with parent module      |
| BONE_VERTEBRA_KNEE_BUSHING_PILOT  | 12 mm      | TBD      | distal end           | Validate with distal module      |

## Simulation fields

- Approximate mass (wood): see project calculation
- Center of mass: ~45% length from proximal end (long bones)
- Joint axis: see project joint module
- Joint limits: as per joint module

## Source datasets

- Build values: [`profiles/adult_female_21_28.json`](../profiles/adult_female_21_28.json)
- Biological values: [`skeleton/datasets/female_21_baseline.json`](../../../skeleton/datasets/female_21_baseline.json)

---
