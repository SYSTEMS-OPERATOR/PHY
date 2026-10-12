# Facial canonical source completion: validation evidence

Date: 2026-10-12. Base: PR #106 merge
`d73369127a5905dbd1fdc999041f0e9c908145ad`.
Branch: `agent/phy-facial-source-records`.

## Selected mission and result

Close the canonical source discrepancy without adopting unsupported physical
data. Add the 13 facial identities already present in the display inventory,
using identity-scoped publisher evidence and explicit unknowns. Canonical source
coverage is now **206/206**, with zero missing source identities. Individual
display coverage stays 206. Reference declarations increase from zero to 13;
complete declared dimension maps remain 36, dataset bindings 21, dimensional
primitive estimates 30 and resolved anatomical-reference sets 24.

The new records have unresolved Latin nomenclature and shape classification,
null dimensions/density/mass/inertia, empty source geometry, null parents and no
declared articulations, mounts or joint interfaces. The existing BODY tolerance
default is explicitly a policy placeholder. The local OpenStax catalog supports
identity names and paired/unpaired accounting only.

All previous 193 source files and canonical source records are preserved. Their
published canonical snapshot fingerprint remains
`4b71c8a1c64cb8eeea8a1032a714d95ef32c7322391183522b206d6990517597`.
All 230 display parts preserve their geometry and other metadata; only source
presence/path fields change for the 13 new records. No SOPHY canon, stature/span,
profile, schema, root authority file, A0 article or maquette is changed.

Current Studio metadata, portable HTML and generated coverage reports reflect
source completeness while retaining provisional geometry. The cached canonical
exports are refreshed using the already-merged validation-aware exporter as an
explicit review package, including the manifest and export report. This also
replaces older unmanifested cache formatting with the current exporter contract;
the exporter implementation and source measurements are unchanged.

## Validation results

| Check | Verified result |
| --- | --- |
| Final full local Python 3.12 pytest suite | **267 passed, 5 skipped, 951 subtests passed** |
| New source regression tests | All **nine tests** pass in the full suite |
| Standard-library source regression suite | **Nine unittest tests passed**, without pytest/CadQuery |
| Old 193 source files and canonical data | Original source-file and canonical-record fingerprints match |
| Display preservation | Complete 230-part fingerprint matches after excluding only source-presence fields; earlier 199-proxy golden snapshot also preserved |
| Canonical review cache | 206 records; source fingerprint and all nine artifact hashes match manifest; readiness/release false |
| Existing fabrication validator | Expected exit 1; **nine blocker categories**, including 13 unknown densities |
| Both real export/assembly CLIs outside checkout | Default validated mode rejects; explicit review succeeds with 206 records; subsequent rejection preserves prior package |
| Independent audit repeats | All three audit files byte-identical across output roots |
| Viewer syntax/build | Passed |
| Actual Three.js GLB/STL logic | Passed: 206 distinct IDs; all 22 skull source pointers present; provisional provenance retained |
| Two fresh complete Studio exports | All **24 files byte-identical**, valid 23-entry ZIPs |
| Checked-in portable HTML and seven generated reports | Byte-identical to fresh exports |
| Maquette | All six existing files preserved |
| SOPHY canon export | Version 1.0.0 and stature/span 1676.4 mm validate successfully |
| Whitespace and Python syntax | Passed |

The nine new tests independently cover the exact identity set, explicit unknown
values, identity-only evidence scope, old source/record fingerprints, all prior
display geometry, independent audit observations, missing physical evidence,
validated-export rejection, null mechanical transforms and coherent cached
review publication.

The source-material legacy regression now uses a fresh snapshot and requires
all 13 unknown densities to appear in its existing failure report. Prior mutable
runtime selections cannot be used to imply source-material completeness. Neither
the legacy nor fabrication validator is modified.

## Expected gaps and unavailable checks

Loaded unknown-dimension records are now 170, unresolved geometry records 176
and missing dataset bindings 185. All 206 records lack mounts and joint
interfaces; the original 193 still lack references. Physical/articulation
qualification remains open and fabrication release remains false.

Four optional PyBullet-dependent modules and one Docker check were skipped.
Local browser smoke could not launch because Playwright Chromium is absent.
Python 3.11 was unavailable locally. Existing CI retains its browser/Python 3.11
checks and includes the new source regressions in Studio's focused suite.

As observed during the preceding audit mission, an initial default-output ZIP
was incomplete locally. No equality/validity claim is made for that initial
archive. Both fresh independent output roots produced valid, byte-identical
archives; the ignored default cache was restored from a verified repeat and its
CRC/entries rechecked. The archive writer is unchanged. All nonarchive default
outputs matched the independently verified exports.

## Reproduce

```bash
python -m pytest -q
python -m unittest discover -s tests -p test_facial_source_records.py -q
python bin/audit_engineering.py --output-root /tmp/phy-facial-audit
PYTHONPATH=. python bin/export_fabrication.py --mode review --output-root /tmp/phy-facial-review
npm run check --prefix studio
npm run build --prefix studio
python bin/export_phy_studio.py --output /tmp/phy-facial-studio-one
python bin/export_phy_studio.py --output /tmp/phy-facial-studio-two
diff -qr /tmp/phy-facial-studio-one /tmp/phy-facial-studio-two
npm run test:logic --prefix studio
```

Full Studio export requires CadQuery 2.7.0; source loading/audit and the nine
standard-library tests do not. Evidence scope and subsequent regional
relationship work are specified in `docs/facial_source_records.md`. This branch
is prepared for review with manual merge and branch retention.
