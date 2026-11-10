# Spine review validation

Baseline main: `fa02634120438687d67c6d156aa0aec1adf41aa1`.
Reviewed source checkpoint: `cb472b8552a75e96a8a857e9e22979325e2d3900`.

## Reviewed facts and boundaries

- 26 adult spine identities retained; no additional bones or fused segments counted.
- 105 native rows compared with fresh primary-source responses: 20 cervical,
  60 thoracic and 25 lumbar. No numerical transcription mismatch. All 102
  previously admitted feature spans are unchanged; three rows remain quarantined.
- All 206 normalized bone dimension maps exactly match main. The 40/206 complete
  declared-map count is unchanged; it is not a fidelity or qualification score.
- C1 and coccyx superior anatomical references now resolve, with no new signal
  links or mechanical definitions. Qualitative feature citations cover all 26
  spine records. Global reference-declaration coverage is now 47/206 and resolved
  declared-anatomical-reference sets are 26/206; record validation passes stay 0.
- Sacrum 100 x 110 x 10 mm remains unverified legacy dataset input. Coccyx and
  the 24 unfused vertebrae retain whole-bone unknowns. Atlas body absence, dens,
  thoracic facet exceptions and coccygeal fusion variability are explicit.
- Features have no qualified surfaces, dimensions, attachment coordinates,
  joint axes, tolerances or physical evidence. Fabrication release remains false.

Fresh source response and native fact hashes, comparison methods and source
locators are recorded in `references/spine/source_review.json`. The spine CLI
revalidates local fact custody; it does not fetch or authenticate publications.

## Software checks

The focused standard-library suite passed **183 test methods**:

```sh
python -m unittest tests.test_geometry_units tests.test_spine_audit tests.test_dimension_evidence tests.test_engineering_audit tests.test_facial_source_records tests.test_fabrication_validation tests.test_fabrication_exports tests.test_relationships tests.test_relationship_schema tests.test_field_connections tests.test_bone_loading_connections tests.test_bone_equivalence tests.test_studio_evidence tests.test_sophy_geometry_canon -q
```

After repinning Studio to the committed source checkpoint, its ten evidence tests
passed again. Three existing forward-kinematics, inverse-kinematics and positive
inertia test functions also passed, run via `unittest.FunctionTestCase`.
The four new GeometryUnitTest methods reproduced 33 subtest failures and one
error against the original GeometryAgent; they pass with the correction.

New spine checks cover every admitted value and retained unknown, source hashes,
feature/reference drift, superior-reference mistakes, parent cycles, partial and
duplicate inputs, JSON/Markdown manifest hashes, outside-checkout CLI execution,
repeatability, runtime isolation and failed-input preservation of prior reports.
Malformed sizes and conflicting unit aliases are rejected. Body/canal measurements
cannot populate missing whole-bone dimensions or create solid vertebral proxies.

## Package and geometry checks

- Normal review export regenerated with twelve artifacts. Validated readiness and
  fabrication release remain false; the existing gates were not relaxed.
- Engineering, dimension-expansion and spine reports regenerated from fresh data.
  The strict spine consistency check passes while `all_dimensions_verified` is false.
- Studio evidence was explicitly reviewed and repinned. Its source/record hashes
  change; its display-parts fingerprint is unchanged. The stale-evidence gate remains.
- Two full Studio exports using pinned PHY-REF revision
  `1671f55f7a7cd6f03af4497dd02da02906135795` are byte-identical across all 34 files,
  including the package ZIP. The tracked portable HTML matches the generated HTML.
- All seven embedded model objects exactly match a separate export of baseline
  main: F28_MEAN, F28_REFINED, SOPHY_SCALE, A0_R2, REFERENCE_KIT, NIH_PELVIS and
  NIH_SKIN. Only evidence, relationship, manifest and containing-package files change.
- `npm run check --prefix studio` and `npm run test:logic --prefix studio` pass.
  The exact original-surface custody fixture was explicitly skipped because
  PHY_PELVIS_ORIGINAL was not supplied; pinned-preview and original-review logic pass.
- Canon kernel and active T56 profile bytes match baseline exactly. H = span =
  1676.4 mm, deterministic geometry and mirror derivation remain unchanged.

No full repository CI, browser-interaction run, physical measurement, load test,
independent engineering approval or fabrication release is claimed. No source
PDF, publisher image or newly acquired original anatomical surface is committed.
