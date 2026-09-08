# Establishing reference-backed T-5.6 dimensions

## Selected references and fit to the project

Use the US Army's ANSUR II public female measurements as the initial empirical
comparison source, with definitions from **NATICK/TR-15/007, December 2014**.
The primary Army report and public working data were retrieved through Penn
State's OPEN Design Lab mirror. The catalog page labels the report 2015; the
report title page itself says December 2014, which is recorded here. File hashes,
retrieval date, population, measured units, and exact section/page locators are
in [dimensional_sources.json](dimensional_sources.json).

The existing REDWOOD reference targets are close enough to make this a useful
starting point, but their endpoints are incompletely specified. Its 1650 mm
profile is a legacy engineering comparison, not the authority for T56 identity.
The generated [comparison](../reports/REFERENCE_DIMENSIONS.md) scales only those
legacy values by 1676.4/1650 = 1.016 to compare like stature. It does not rescale
the measured people or change either project's profile.

The second reference is Vitruvius, Book III, Chapter I, already named in PHY.
The hand and foot fractions provide a historical contrast. Their disagreement
with this measured female cohort is visible, rather than removed to produce
apparent agreement. Other historical fractions remain unadopted as in canon 1.0.0.

ANSUR is selected for reproducible landmark-based coverage and a height/span
matched subset, not because military women represent all adult women. Army
population bias, posture, respiration, soft tissue, and measurement error remain
relevant. No ethnic filter or inferred demographic trait is used. The 21–28 age
band follows the existing REDWOOD reference; it is an analysis choice, not body
identity. The ±25 mm bands are comparison filters, never fabrication tolerances.

NASA's [Appendix E](https://www.nasa.gov/reference/appendix-e-vol-2/), sections
E.2, E.3 and E.5, is also selected for landmark and motion review. Its source
population and operating assumptions are recorded separately in the catalog.
No NASA table values are imported in this initial tooling. The exact edition
previously used by the project remains unverified.

## Process

1. **Authenticate reference bytes and units.** Import the pinned CSV; reject a
   checksum mismatch, duplicate subjects, wrong population, missing columns, or
   nonfinite dimensions. Check every imported measure's full-sample mean against
   the primary report's rounded female mean. CSV lengths are mm; report tables
   are cm. Self-reported height/weight and race/administrative fields are excluded.
2. **Preserve a reproducible local projection.** Store all 1,986 rows but only age,
   original CSV line number, and ten selected measurements. Pin the projection's
   canonical JSON checksum too. All subsequent calculation works offline.
3. **Select before looking at outcomes.** Use the committed age/stature/span
   filters. Report n and original row provenance. Compare ±10/25/50 mm bands.
   Reject an insufficient primary cohort; never silently widen it or cherry-pick
   people whose limb measurements resemble desired results.
4. **Produce candidate dimensional specs.** Emit marginal median, observed
   p05/p95, a ratio to the selected H, legacy deltas, definition, source locator,
   and explicit mapping restrictions. Provide one actual multivariate reference
   row near the medians as a coherence check. Independent medians are not a
   measured person, and the actual row does not satisfy exact canon span by fiat.
5. **Define and adopt the landmark model.** For each proposed identity dimension,
   record the endpoint IDs, measurement pose, chosen ratio/equation, source and
   selection hashes, and design rationale. Resolve the span budget using actual
   shoulder/elbow/wrist/hand landmarks. Do not add relaxed-arm scalar segments to
   a seated shoulder breadth and call the result a valid T-pose span. Author the
   right side and derive the left by reflection. Changing the identity graph
   requires a reviewed BODY/kernel canon revision and updated invariant tests.
6. **Derive the build dimensions.** After landmark adoption, compute joint-center
   distances in explicit frames, then account for cartridge offsets, insertion
   depths, clearances, materials, and tolerance stacks in the embodiment overlay.
   Validate through the merged geometry contracts. Release drawings only after
   mission loads, mechanisms, numeric acceptance criteria, and article evidence
   are closed through the existing convergence process.

The new tool implements steps 1–4. Steps 5–6 are explicit review/mapping work;
it has no command to silently adopt a canon law or write a locked geometry record.

## Semantic mapping boundaries

- Biacromial breadth measures acromion-to-acromion, not shoulder joint centers.
- Acromion-radiale and radiale-stylion are external segment measurements. They
  are not osteometric bone lengths, nor cartridge-center distances.
- Bicristal breadth is not hip-joint spacing. Existing `pelvis_breadth` needs a
  named endpoint definition before adopting this value.
- Chest breadth/depth include a specified measurement level, posture, tissue and
  breathing protocol. They cannot be copied into structural housing envelopes.
- Hand/foot measurements are useful nominal external-size candidates once the
  intended wrist/heel/toe endpoints are confirmed. Their constituent bone lengths
  and joint centers remain a separate model.
- ANSUR `headlength` means front-to-back head dimension, not vertical head height.
  No head, ulna, femur, tibia, or fibula value is fabricated from another column.

## Offline tooling

From the repository root, regenerate the initial specs:

```sh
python -S PROJECTS/T56_CARBON/tools/reference_dimensions.py \
  --output PROJECTS/T56_CARBON/reports/reference_dimension_specs.json \
  --markdown PROJECTS/T56_CARBON/reports/REFERENCE_DIMENSIONS.md
```

To reproduce the projection from a downloaded public source:

```sh
python -S PROJECTS/T56_CARBON/tools/reference_dimensions.py \
  --import-csv /path/to/ANSUR_II_FEMALE_Public.csv \
  --output /tmp/ansur_female_dimensions.json
```

Download URL: <https://tools.openlab.psu.edu/publicData/ANSUR_II_FEMALE_Public.csv>.
The tool itself performs no network requests. `--catalog`, `--policy`, `--profile`,
`--legacy`, and `--snapshot` permit explicitly reviewed alternatives. A new raw
file or projection needs reviewed catalog hashes; a changing upstream URL cannot
silently change the build target. Reports include canonical content hashes for
every input, no runtime timestamp, and deterministic row ordering.

The output's `reference_ratio_to_H` is a proposed comparison ratio, not an
adopted proportional law. `fabrication_value_mm` remains null and
`canon_adopted`/`fabrication_ready` remain false. Sample percentiles are not
confidence intervals or manufacturing tolerances. Source evidence and selection
uncertainty are preserved for the eventual dimensional decision.

## First decisions supported by this evidence

Review approximately **369.5 mm acromial breadth, 183 mm external hand length,
and 250 mm external foot length** as candidate reference sizes at the T56 target.
These marginal values are the starting comparison, not an already solved body.
For the shoulder article, prioritize defining the anatomical-to-cartridge mapping
and relaxed-pose-to-T-pose transformation before trying to lock `GEO-ARM-001`.
ANSUR alone cannot determine the posterior/anterior root coordinates, clavicular
mechanism, shoulder trajectory, or structural load limits.

Tests:

```sh
python -S -m unittest discover -s tests -p 'test_reference_dimensions.py' -v
python -S -m unittest discover -s tests -p 'test_t56_geometry_contracts.py' -v
python -S -m unittest discover -s tests -p 'test_sophy*.py' -v
```
