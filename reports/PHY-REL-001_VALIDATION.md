# PHY-REL-001 validation

Base: `de0b827f2fb322b0109509ec02f84e3df1662c4b`, after PR #121.
Scope: typed relationship observations, shared resolution and offline inspection.
No anatomical measurements, proxy geometry, canon laws or A0 inputs changed.

## Current snapshot

| Observation | Result |
| --- | --- |
| Adult canonical identities present | 206 / 206; no missing or supplemental records |
| Legacy parent declarations | 206, including unspecified/unresolved values; these are not 206 mechanical parents |
| Legacy adjacency / articulation declarations | 11 / 11; original values retained |
| Curated TMJs | Two; both share BONE_MANDIBLE; classification synovial; qualitative rotation and translation; engineering definitions null |
| Structural participation | Seven region hyperedges; no generated pairwise contacts |
| Operational links | 16 directed runtime entries, separately identified |
| Hyoid | Parent unspecified; no fabricated articulation or runtime link |
| Legacy mandible | Original temporal/hinge entry retained; exact target missing; runtime matching ambiguous between the two temporal IDs |
| Fabrication release | False; existing record/identity gates retained |

Relationship JSON SHA-256:
`18cba5b821285bcdab92c843905468b60cf03dc87cbca29018cfbaa12d660c13`.
The normal review export and Studio ship identical relationship JSON bytes.

## Verification

- **156 Python tests and 597 subtests passed** across relationship/schema,
  loading, field, export/publication, signal, craniofacial, Studio evidence,
  engineering audit, fabrication validation, Studio geometry and canon checks.
- Draft 2020-12 schema validation covers full/empty partial snapshots and rejects
  engineering adoption, unsupported relationship kinds and extra adult counts.
- Standard-library CLI runs with `python -S` from outside the checkout. Repeated
  graph packages match byte for byte; injected publication failures restore the
  previous package. Invalid evidence cannot replace accepted artifacts.
- `npm run check --prefix studio` and `npm run test:logic --prefix studio` passed.
- `studio/tests/relationships-browser.mjs` passed on desktop and 390px mobile:
  both TMJs, original hinge/ambiguity, source evidence, participant highlighting,
  regional/runtime separation, hyoid, offline operation and review JSON retention.
- Existing `studio/tests/smoke.mjs` passed: offline WebGL, all seven models,
  camera/layout stability, bone filtering, surface picking, ruler, pose/height,
  GLB/STL/JSON and embedded maquette package.
- Two complete Studio exports, including their ZIPs, matched with `diff -qr`.
  Checked-in HTML matched the generated HTML, and the maquette remained unchanged.
- All seven model payloads matched main exactly, including geometry and limits.
  Their combined canonical JSON SHA-256 is
  `7601ae8f5a887e33e9b21b54f40633bad66efe7379fb441e0aeac939f5596faa`.
  Reference data and the canonical reference were unchanged. Canon tests retain
  H = span = 1676.4 mm, deterministic export and mirror-derived symmetry.
- Source whitespace checks and a generated-viewer whitespace scan passed.

Python regression command:

```bash
python -m pytest tests/test_relationships.py tests/test_relationship_schema.py \
  tests/test_bone_loading_connections.py tests/test_field_connections.py \
  tests/test_fabrication_exports.py tests/test_signal_propagation.py \
  tests/test_craniofacial_audit.py tests/test_studio_evidence.py \
  tests/test_sophy_geometry_canon.py tests/test_engineering_audit.py \
  tests/test_fabrication_validation.py tests/test_phy_studio.py \
  tests/test_sophy_project_canon_bindings.py -q
```

## Limits and remaining evidence

The exact original-surface custody fixture was **skipped locally** because
`PHY_PELVIS_ORIGINAL` was not supplied. Existing pinned native previews and
synthetic original-review logic passed. The original-surface protocol was not
changed; CI provides its own original fixture.

The pinned Playwright browser download returned invalid archives. Local browser
checks used Playwright 1.63.0 with Chromium **153.0.8010.0** from an explicitly
configured local executable. A full Git whitespace check once crashed on the
large generated viewer; source diffs and a direct generated-text scan passed.
Full repository CI has not been run locally or awaited.

Identity resolution, content custody and display navigation are software checks.
Engineering definitions, anatomical geometry, physical behavior and fabrication
qualification remain unresolved. No unmeasured evidence is marked complete.
