# Engineering evidence audit

PHY has 206 individual adult display identities, but that does not describe the
maturity of its canonical source data or mechanics. This audit joins the adult
identity inventory, a selected canonical snapshot, provisional reference-body
proxies and the existing validators. It exposes the separate observations and
the evidence still needed for each identity. It does not change a validation gate.

## Run offline

```bash
python bin/audit_engineering.py --dataset female_21_baseline --output-root /tmp/phy-audit
```

The entrypoint works outside the checkout and requires only Python's standard
library. It constructs fresh canonical records and the default F28 reference
body. SOPHY canon 1.0.0 and its 1676.4 mm stature/span remain unchanged; the
separate A0-R1 engineering article is outside the audit's scope.

Outputs under the selected root:

- `reports/engineering_audit.json`: independent totals, regional totals, all 206
  identity rows, detailed existing record/identity validation, and qualification
  limits.
- `reports/engineering_audit.md`: readable observations and a complete identity
  crosswalk.
- `reports/engineering_audit_manifest.json`: SHA-256 hashes of both reports and
  their input fingerprints.

Exit 0 and `completed: true` mean the observation report was produced, including
known gaps. They do not mean the records pass validation or fabrication is
released. Input or publication errors exit 1. Failed input checks do not replace
the previous report. Publication reuses the exporter's staging/rollback utility;
readers must verify the manifest hashes, and callers must give it exclusive
access to the destination. There is no atomic transaction across all three files
under process or power loss.

## Shared inventory

`skeleton/anatomical_inventory.py` owns the existing `PHY_ADULT_206_V1` identity
list, regional accounting and static source-module lookup. Its contents were
moved from `skeleton.visualization.bone_equivalence` without changing IDs or
counts. The original import locations remain available for existing consumers.
The inventory contains no new BoneSpec records, measurements or geometry.

Source-module presence means a local file declares the ID. Loaded canonical
record presence means the supplied snapshot contains that ID. The audit keeps
those counts separate: a partial snapshot does not acquire records just because
their source files exist. The API can inspect selected snapshots; source lookup
alone does not establish that a caller's supplied object was loaded from that
file. The CLI uses `load_field` to construct its snapshot from current sources.

## Independent observations

All totals use the 206 adult identities as their denominator; each region uses
its own inventory size. Hardware, grouped forms and missing records cannot
inflate an observation. There is no combined equivalence score.

| Observation | Meaning and boundary |
| --- | --- |
| Individual display identity | One provisional reference-body proxy for the ID. Morphology is still unverified. |
| Source module present | Static ID-to-file correspondence. Its SHA-256 hash records the source bytes inspected. |
| Canonical record present | The loaded snapshot contains the expected ID. Missing records stay missing. |
| Declared dimensions complete | A nonempty normalized dimension map contains only finite, positive numbers. This is completeness of declared values, not evidence that the dimensions are correct or anatomically sufficient. |
| Dataset binding present | The record's selected dataset has an object at its declared key. Binding is independent of whether it supplies dimensions or has qualified provenance. |
| Reference declaration present | Nonempty reference strings are declared. Citations are not fetched, authenticated or qualified. Self-derived source IDs are not treated as references. |
| Dimensional primitive available | Existing canonical geometry has `dimensional_estimate` status. A box/plate estimate is not a validated anatomical surface. |
| Declared anatomical references resolved | At least one declaration exists, all declarations resolve, and no parent cycle involves the ID. Unspecified relationships are not counted as resolved. Anatomical adjacency remains distinct from a mechanical transform tree. |
| Record validation pass | No per-record blockers in the existing ValidatorAgent. Whole-model identity validation and physical acceptance remain independent. |

Per-identity dimension states are `complete`, `partial`, `unknown` or
`missing_record`. Null values remain null, and invalid/nonpositive dimensions
are listed separately from unknown values. `values_mm` is the canonical record's
normalized dimension map; unit-contract errors remain in record validation.
Metric-source declarations retain their original keys and dataset bindings.

Proxy geometry is never used to populate source dimensions. Each identity keeps
its provisional proxy disposition and geometry-input pointers alongside the
canonical geometry's independent status. No assembly pose, transform, joint
type, floating-bone suspension or measured physical behavior is synthesized.

The audit labels dimensional fidelity unverified, anatomical geometry and
articulation unqualified, mechanical transforms unresolved, physical evidence
unmeasured and fabrication release false. These labels describe this reference
pipeline's lack of an evaluated qualification contract. They are not numerical
coverage scores or an assessment of the separate A0 article.

`evidence_needed` is a list of work categories, not prescribed values or a
mechanical checklist. In particular, a missing/undeclared hyoid relationship
calls for anatomical review; it is not a request to invent a fixed parent.

## Determinism and validation

Record ordering and proxy fingerprint ordering are stable by ID. The report has
no timestamp, absolute checkout path or output-root-dependent bytes. Input
fingerprints include the shared inventory, strict canonical record snapshot,
full supplied display parts and selected datasets. Each source row includes
its relative path and file hash. The canonical fingerprint uses the same JSON
serialization and ordering as `export_manifest.json`'s
`source.source_records_sha256`, allowing the two reports to be compared.

This is a stable-checkout observation, not an atomic filesystem snapshot.
Do not edit source modules, datasets or profiles while constructing a report.
Unknown, duplicate or unsupported adult IDs are refused. Nonconvertible and
nonfinite canonical inputs are refused with the offending identity in the CLI
diagnostic; the existing fabrication validator remains available for detailed
malformed-record diagnostics.

Regression checks cover real checkpoint counts, independent regional totals,
canonical-export fingerprint agreement, partial snapshots, unknown measurements,
proxy changes, dataset binding gaps, side ambiguity, cycles, malformed children,
invalid IDs, nonfinite inputs, runtime isolation, source lookup failures,
manifest hashes and CLI execution/failure recovery outside the checkout.

## Verified initial checkpoint

Base: `c4792671c446033185de2953b8786e5ed508b876`, the PR #105 merge.
Dataset: `female_21_baseline`. Default F28 display proxies.

| Observation | Count / 206 |
| --- | ---: |
| Individual display identities | 206 |
| Canonical source modules | 193 |
| Loaded canonical records | 193 |
| Complete declared dimension maps | 36 |
| Selected dataset bindings | 21 |
| Reference declarations | 0 |
| Canonical dimensional primitive estimates | 30 |
| Resolved declared anatomical-reference sets | 24 |
| Records passing existing record validation | 0 |

The remaining 157 loaded records have unknown dimensions; 13 facial identities
lack canonical records. Canonical geometry is unresolved for 163 loaded records.
There are 172 unresolved reference declarations in the identity resolver and
172 missing dataset bindings. All 193 loaded records lack references, joint
interfaces and mount points. Eight record-blocker categories remain. Source
presence and complete declared dimensions do not resolve those blockers.

## Next bounded engineering work

Introduce the 13 missing facial canonical identities with explicit unknown
geometry/dimensions and supported identity references, using this audit to prove
that source coverage improves while dimensional/mechanical qualification stays
open. Then review explicit anatomical relationship semantics in a small region
before replacing legacy name references: sutures, bone articulations, cartilage
and soft-tissue suspension need different declarations. Stable IDs alone do not
make a correct kinematic hierarchy. New qualification claims need explicit,
independent contracts and evidence rather than a new completeness percentage.
