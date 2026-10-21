# PHY Studio: original-surface pelvis review

Directive PHY-DEV-PELVIS-REVIEW-001, executed against PHY main
`b936b6c7db329408e4da0f4cfb91532201ce3433` and PHY-REF main
`f2f9ffcb7bd0970506eb42b2e325eec1a89ffe71`.

The Studio consumes the immutable tested PHY-REF tooling commit
`1671f55f7a7cd6f03af4497dd02da02906135795`, proposed in
[PHY-REF PR 3](https://github.com/SYSTEMS-OPERATOR/PHY-REF/pull/3).
Its [integrity/geometry run](https://github.com/SYSTEMS-OPERATOR/PHY-REF/actions/runs/39518957955)
passed, including explicit original acquisition and independent review tests.

## Delivered behavior

The seven existing offline models remain. An explicit local-file selection can
add a session-only original pelvis after exact byte-length, header, hash and
supported geometry verification: 14 meshes, 87,448 original triangles. The
ordinary build still reads only two preview GLBs. No original is embedded,
automatically fetched by Studio, transformed into PHY, or adopted as a proxy.

Original capture records exact source node, primitive, original face, vertex
indices, barycentric weights and unchanged native world meters. The renderer
uses the original index order and the complete source transform chain, followed
only by the existing 1000 mm/m display conversion. Patch editing records explicit
original faces and documented rules. Missing constructions remain unresolved;
no representative point or anatomical plane is inferred.

Versioned original drafts and packets can resume after the matching original is
reloaded. Complete validation precedes replacement; invalid inputs leave active
work intact. Preview drafts retain their separate existing version. Comparison
keeps A and B intact, identifies missing features/rule/patch differences, and
labels distances as converted source-coordinate differences with calibration
and an approved uncertainty budget unresolved. Declared reviewer IDs do not
establish independent human evidence. No average or tolerance verdict is issued.

Packets contain source/definition/contract/revision/build fingerprints, one or two
reviews, independently reproducible reconstruction results, comparison, review
notes/views, attribution, CC BY 4.0 and unresolved gates. Optional annotated PNGs
have separate rendering-context sidecars and are not geometric proof. JSON has
recursive sorted object keys, preserved array order, two-space UTF-8 formatting
and a final newline. Identical exports from one producer are reproducible;
JavaScript/Python numeric formatting need not be byte-identical.

## Verification

| Check | Local evidence |
| --- | --- |
| Reference validator | 26 unittest tests passed with explicitly supplied exact original |
| PHY regression selection | 215 tests and 937 subtests passed; includes Studio, source custody, canon, geometry and fabrication regressions |
| Source/bone/native/original logic | Syntax/build and logic checks passed; all five original surfaces reconstructed, parent transforms exercised, tampered bytes and prohibited claims rejected |
| Original browser | Five real clicks on original triangles; two-face patch add/remove; candidate replacement/deletion; strict invalid-import rollback; draft/packet reopen; distinct fixture comparison; mobile width and bounded canvas checks; zero HTTP requests/errors; annotated PNG and rendering-context sidecar verified |
| Independent cross-check | Browser draft A, draft B and exported packet passed PHY-REF direct-buffer reconstruction |
| Preserved files | All 23 recorded baseline hashes unchanged, including authority/schema files and 19 pre-existing geometry/layout/maquette payloads |
| Determinism | Repeated packet exports matched byte for byte; fresh distribution rebuild matched and checked-in HTML is synchronized |

All browser points are **software fixtures**, chosen on arbitrary visible
triangles to exercise picking. They do not locate the named anatomical features.
`reports/fixtures/pelvis-original-software-packet.json` is the concrete replay
packet. A and B share four points and deliberately differ at the fifth to test
comparison. These are not independent anatomical observations.

The local Playwright browser download returned an invalid zero-length ZIP.
Local browser checks used `@sparticuz/chromium@153.0.0` with a temporary
single-process SwiftShader wrapper. The repository retains the normal Actions
Playwright launch. Standard Actions results are an additional handoff gate;
see the linked PR checks and `phy-studio-demo` artifact for screenshots and the
independently validated fixture packet produced in that environment.

## Reproduce and inspect

From PHY, with an explicit pinned PHY-REF checkout:

```bash
npm ci --prefix studio
npm run check --prefix studio
npm run build --prefix studio
python bin/export_phy_studio.py --reference-root /path/to/PHY-REF --sync-review
PHY_REF_ROOT=/path/to/PHY-REF python -m pytest tests/test_studio_references.py -q
PHY_PELVIS_ORIGINAL=/path/to/VH_F_Pelvis.glb npm run test:logic --prefix studio
PHY_PELVIS_ORIGINAL=/path/to/VH_F_Pelvis.glb npm run test:browser --prefix studio
python bin/export_phy_studio.py --reference-root /path/to/PHY-REF --output /tmp/phy-repeat
diff -qr studio/dist /tmp/phy-repeat
cmp studio/PHY-Studio.html studio/dist/PHY-Studio.html
```

From the pinned PHY-REF checkout, independently:

```bash
PHY_PELVIS_ORIGINAL=/path/to/VH_F_Pelvis.glb python -m unittest discover -s tests -v
python tools/validate_original_review.py --original /path/to/VH_F_Pelvis.glb --packet /path/to/PHY/reports/fixtures/pelvis-original-software-packet.json
```

The new original CLI requires the packet's exact reference revision, source and
contract. The existing preview CLI remains unchanged. Without explicitly supplied
original bytes, original integration checks report skips and perform no download.
The normal Actions workflow acquires the exact pinned source for explicit test
file inputs and independently checks its browser output.

## Preserved authority and remaining evidence

All 206 individual bone identities and 40/206 complete declared dimension maps
remain. SOPHY 1.0.0 height/span stay 1676.4 mm. The original and preview bytes,
source asymmetry, fused adult left hip identity, maquette and A0 bearing/stop
blockers remain unchanged. No canonical authority/schema was edited.

Accepted landmarks and canonical transforms remain null; anatomical acceptance
and fabrication release remain false. Independent anatomical reviewers, metric
calibration, segmentation review, approved uncertainty budgets, anatomical
adoption/registration and source L6 context remain unresolved. This pass delivers
review software; it supplies no anatomical or fabrication acceptance.
