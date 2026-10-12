# NASA human anthropometry reference acquisition

Status: source identified; binary PDF not yet mirrored or hash-verified.

Source: NASA Human Integration Design Handbook (HIDH), OCHMO-HB-004.
Official landing page: https://www.nasa.gov/human-integration-design-handbook/

Scope: obtain the official Anthropometry, Biomechanics, and Strength chapter, including female-specific data where explicitly identified. Retain document revision, URL, retrieval date, exact byte size, SHA-256, and source license/publication statement before committing the PDF. Compare against existing `PROJECTS/PHY_F28/references/ansur_age28.json` and `PROJECTS/T56_CARBON/references/ansur_female_dimensions.json` without silently combining populations or modifying canon.

Storage decision: the NASA PDF belongs directly in PHY, near the existing `references/` assets; do not import the complete handbook unless independently justified. A PDF is not a female 3D mesh. The existing tree audit found no NASA-named file in PHY on 2026-10-12.

Acceptance: official binary obtained, checksum recorded, source/revision and rights checked, sex-specific tables identified, and a crosswalk produced with units and cohort labels. Until then this is an acquisition record, not a claim of completed backup.
