# Facial canonical source identity completion

Base: PR #106 merge `d73369127a5905dbd1fdc999041f0e9c908145ad`.
Branch: `agent/phy-facial-source-records`.

The display inventory already contains 206 adult identities. Thirteen facial
identities lacked canonical BoneSpec records: left/right maxilla, zygomatic,
nasal, lacrimal, palatine and inferior nasal concha, plus the unpaired vomer.
The existing adult mandible record is preserved.

## Evidence boundary

OpenStax's *Anatomy and Physiology 2e*, section 7.2, identifies six paired facial
types and the unpaired vomer and mandible. The publisher's page was reviewed on
2026-10-12. The local catalog
`references/facial_identity_provenance.json` maps that identity reference to
each new source file, with explicit supported and unqualified fields:

https://openstax.org/books/anatomy-and-physiology-2e/pages/7-2-the-skull

The reference supports identity names and paired/unpaired accounting. It
supplies no adopted numerical dimensions, surfaces, density, mass, inertia,
interface design or assembly transform for these records. Latin nomenclature
is explicitly `unresolved`, and shape classification is `unclassified`; neither
is inferred from the visual proxy. No functions, developmental details or
complete adjacency graph are adopted. The local catalog is a curated declaration
of reviewed identity evidence, not an archived full source page or automated
citation qualification.

## Records and integration

Each new `skeleton/bones/bone_*.py` module contains one literal, stable BoneSpec
identity so both the canonical loader and static inventory lookup discover it.
The established adult IDs and English display names are preserved.

- Dimensions are three canonical mm keys with null values.
- Material density, mass and inertia remain null. Generic default density is
  not substituted for missing evidence.
- Source geometry is empty; canonical conversion explicitly reports
  `unresolved` / `no source geometry`.
- Parent is null, children and articulations are empty, and no joint interfaces
  or mounts are declared. These omissions do not qualify mechanical topology.
- Source IDs and references link to the identity-only local catalog/publisher.
- The existing 0.5 mm BODY tolerance default is retained and explicitly labeled
  a policy placeholder, not a measured or qualified manufacturing fit.

Studio's 13 facial proxies now report source-record presence. Their vertices,
faces, dimensions, pose inputs, topology hints and qualification labels are
preserved. The coverage dialog and current generated reports distinguish source
identity completeness from unresolved physical data. The portable HTML and
actual GLB export metadata carry that same distinction. The source-dimensioned
component gallery and separate A0-R1 article do not gain speculative solids.

The previous 193 source files and selected-dataset canonical records are
unchanged. SOPHY canon 1.0.0, 1676.4 mm stature/span, project geometry profiles,
authoritative schemas, root authority documents and all six maquette files are
preserved. Existing validation and publication code is unchanged.

The checked-in canonical JSON, TF/URDF inventories, BOM/tables, reference audit
and validation reports are refreshed as an explicit **review** package. Its
manifest and export report declare 206 records, readiness false and fabrication
release false, with verified hashes. These caches now agree with the source
inventory; refreshing them does not qualify the records.

## Measurable result

Run `python bin/audit_engineering.py --output-root /tmp/phy-facial-audit`.
Counts use the same independent observation definitions introduced by PR #106.

| Observation | Before | After |
| --- | ---: | ---: |
| Individual display identities | 206 | 206 |
| Source modules / canonical records | 193 | 206 |
| Missing canonical identities | 13 | 0 |
| Complete declared dimension maps | 36 | 36 |
| Dataset bindings | 21 | 21 |
| Reference declarations | 0 | 13 |
| Dimensional primitive estimates | 30 | 30 |
| Resolved declared anatomical-reference sets | 24 | 24 |
| Records passing the fabrication record gate | 0 | 0 |

Loaded records with unknown dimensions rise from 157 to 170; unresolved geometry
from 163 to 176; missing dataset bindings from 172 to 185. All 206 lack mounts
and joint interfaces; the original 193 still lack references. Thirteen unknown
densities introduce the existing `material_incompleteness` blocker category,
raising the record-blocker total from eight to nine. This is explicit unknown
data, not increased physical readiness.

Validated export remains blocked, review export contains 206 identities with
null mechanical transforms, and every package retains fabrication release
false. A legacy runtime material-roundtrip check also reports the new unknown
densities. Its regression uses a fresh source snapshot so prior mutable runtime
material selections cannot hide those gaps. No legacy validator is modified.

## Verification and next work

`tests/test_facial_source_records.py` independently checks the exact 13-ID set,
scope of evidence, explicit unknowns, old source-file/record fingerprints, all
230 prior display parts, audit observations, unchanged gates and review export
transforms. Existing inventory, skull, ear, audit and actual GLB/browser tests
now expect 206 source identities while retaining provisional geometry status.
Historical scope and validation checkpoints remain historical.

The next bounded work should define anatomical relationship semantics for a
small skull region, with identity references and explicit distinctions between
sutures, articular adjacency and mechanical interfaces. Do not turn an apparent
display contact or the generic immobility of skull bones into a fixed-joint
engineering transform. Measurement and morphology acquisition can mature
independently, with an explicit evidence contract for each adopted value.
