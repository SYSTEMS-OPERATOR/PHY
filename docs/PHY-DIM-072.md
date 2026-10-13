# PHY-DIM-072 — load-bearing dimension evidence

13 October 2026. Execution record for the assignment to expand complete declared
dimension maps from 36 to at least 72 of the existing 206 identities. Evidence
requirements take precedence over the numerical target. The assignment permits
delivery of the maximum defensible result with an explicit shortfall.

**This slice reaches 40/206 complete maps (19.4%), with four new complete maps
and 28 improved partial maps. It does not reach 72/206. The shortfall is 32 maps.**
There are 93 newly traceable fields across 32 candidates. The other four
candidates remain unresolved. All 36 previously complete dimension maps retain
their values. The 24 selected vertebrae retain their original unknown whole-bone
length, width and thickness: body heights and canal clearances are separately
named, never substituted into these fields.

## Authority and baseline

| Input | Pinned revision / result |
| --- | --- |
| PHY main | `2064af478d6554f66900f655afdd64c8869a16b5` |
| PHY-REF main | `5d4cd1e841ec275580e6634fee760e9615f02c30` |
| Fresh engineering audit | 206 identities/records, 36 complete maps, 170 unknown maps, 510 unknown dimension fields, zero record-validation passes |
| Physical authority | BODY.md; existing BoneSpec, schemas and SOPHY canon |
| SOPHY reference | Canon 1.0.0, height/span 1676.4 mm; unchanged |
| Baseline dataset | female_21_baseline; distinct from F28/ANSUR II projections and all new study cohorts |

The snapshot in `references/dim072/baseline.json` records all 206 original maps,
their missing fields, source-module hashes and independent metrics. Its hash is
pinned in the evidence catalog. This is a historical comparison input, not a
replacement dataset or geometry authority.

The existing 36 populated maps are legacy declarations, not newly qualified
anatomy. Sacrum, sternum, both hip bones, long bones and shoulder-girdle records
already count. They were excluded from additions. Numerical defaults in
female_21_baseline and authored display proxies were not reused as evidence.

## Selected candidates

| Region | Previously incomplete candidates | Count |
| --- | --- | ---: |
| Lumbar spine | L1–L5 | 5 |
| Thoracic spine | T1–T12 | 12 |
| Cervical spine | C1–C7 | 7 |
| Foot support chain | Talus, calcaneus and navicular, both sides | 6 |
| Knee loading | Both patellae | 2 |
| Thoracic / shoulder-root context | Ribs 1–2, both sides | 4 |
| Total | Unique IDs from the existing inventory | 36 |

This replaces the provisional allocation because sacrum and sternum are already
complete, shoulder-girdle records already count, and coccyx is less important to
the standing axial load path than the ankle/hindfoot. The ranked 36-row matrix
and all changed values are in `reports/dimension_expansion.md`; the JSON report
compares every original and resulting map, including the unchanged identities.

Four complete additions: bilateral talus and patella. Talus dimensions are
side-specific female CT statistics. Patella height/width are explicitly pooled
female statistics proposed as bilateral defaults; thickness remains
side-specific. The older patella cohort is disclosed and is not a 28-year-old
population claim. The article's millimeter row printed as “Patella weight” is
interpreted as width using its PW definition, methods and width cross-check;
the editorial interpretation is recorded in the catalog.

Partial improvements: all 24 vertebrae, both calcanei and both first ribs.
Calcaneal body width and wall height are local spans, so whole-bone width and
thickness remain null. First-rib chord/arc starts at the tubercle and excludes
neck/head; local shaft diameters do not resolve complete rib extents. No
admitted numerical defaults were found for the naviculars or second ribs in
this bounded review.

## NASA review and positional / motion crosswalk

All three archived PDFs were read locally and independently rehashed. The
hashes match their existing source sidecars. No NASA bone-length or
vertebra-dimension defaults were inferred from these handbooks.

| Archive / verified revision | Relevant locator | Use within PHY |
| --- | --- | --- |
| OCHMO-HB-004 Revision A; approved 2023-12-20 | Section 3, PDF/printed pages 11–13 | Specify posture, landmarks, device alignment and measurement repeatability. Retain different cohort/percentile relationships; never subtract external heights to invent bone spans. |
| HIDH NASA/SP-2010-3407/REV1; Revision 1, June 5, 2014 | Section 4.2, PDF pages 47–48 / printed 42–43 | Population projection/regression guidance for external dimensions. It does not supply the internal skeletal measurements required here. |
| HIDH Revision 1 | Table 4.3-4 and Figure 4.3-2, PDF pages 63–64 / printed 58–59 | Distinguish 1g reference measurements from 0g posture and spinal elongation accommodation. These are human-envelope effects, not permission to resize SOPHY or its vertebrae. |
| HIDH Revision 1 | Section 4.4, PDF page 67 / printed 62 onward | ROM and motion definitions are task/posture-dependent. Unsuited, unpressurized and pressurized suit conditions remain distinct from a mechanical armature's achievable motion. |
| HIDP NASA/TP-2014-218556, September 2014 | Section 4.5, PDF page 57 / printed 4-30 | Integrated accommodation, iterative analysis and physical evaluation; informs the later interface-review method. |
| HIDP | Section 4.5.3, PDF page 77 / printed 4-50 | Define parent/child rotation and coordinate convention before comparing ROM. No numeric mechanical joint limits are adopted by this slice. |
| NASA-STD-3001 Vol. 2 online Appendix E | E.2 and E.3, inspected 2026-10-13 | External anthropometric projections and condition-specific ROM context; kept separate from individual bone measurements. |

Archive SHA-256:

- OCHMO-HB-004: `ae38d30adbe9b8dcedb89874dcc246d48e834c24bc54d1cfd770d5594c0ee4b5`
- HIDH Revision 1: `a6a8758b9116984d62fe4b7bc31fa9cf5da2479d57b7da9a3e31a00385ed303f`
- HIDP: `04a43994f40f6652eb74bcdda721d83c0534afaced750e429cc9df317bbfcef6`

[NASA Appendix E](https://www.nasa.gov/reference/appendix-e-vol-2/) identifies
its anthropometric basis as 1988 ANSUR projected to 2015. This is distinct from
PHY's ANSUR II projections and F28 selection. Measurements involving the spine
require condition-specific accommodation in NASA's human-interface context;
none of those allowances is applied to bone defaults here.

Positioning work therefore remains an explicit dependency: establish anatomical
landmarks and article datums, retain the native measurement axes, then review a
transform to BODY.md's subject-right/anterior/superior frame. No joint center,
mount, cartilage thickness, posture transform or interface clearance is
manufactured from an external body landmark. NASA guidance is reference
evidence, not automatically a binding PHY requirement or compliance claim.

## NIH candidate disposition

NIH entry pages and their embedded public metadata were inspected. This resolves
some metadata that was unavailable in the previous checkpoint; it does not
qualify the meshes. Entry versions are separate from HRA reference-object
versions and source filenames.

| Entry | Verified metadata | Disposition |
| --- | --- | --- |
| [3DPX-020984, Pelvis/Female](https://3d.nih.gov/entries/3DPX-020984) | Entry v2; HRA pelvis v1.2; input `VH_F_Pelvis.glb`, file ID 2107981, reported 1,593,336 bytes; CC-BY link resolves to 4.0; Visible Human lineage, Browne/Schlehlein attribution | Original binary retrieval returned HTTP 403. No byte hash, actual segmentation, axes, bounds or metric fidelity verified. Not admitted for measurements. |
| [3DPX-020992, Body/Female](https://3d.nih.gov/entries/3DPX-020992) | Entry v2; HRA united-female v1.5; input `3d-vh-f-united.glb`, file ID 2107999, reported 211,624,968 bytes; CC-BY 4.0; Visible Human lineage | No full binary materialization. Organ/body label does not establish individual skeletal segmentation, native metrics or a measured full skeleton. Not admitted for measurements. |
| [3DPX-014841](https://3d.nih.gov/entries/14841?version=1) | Entry v1; metadata declares printUnits mm, anonymized-patient lineage, individual L4/L5 and pelvic files; CC-BY-NC-SA; teaching photographs describe 130% prints | File-size and filename metadata inspected, but raw scale/frame and fidelity not independently verified. Combined sacrum/coccyx asset does not resolve separate bones automatically. No measurements or mesh redistribution. |
| [3DPX-015843](https://3d.nih.gov/entries/15843?version=1) | Entry v1; remix of 014841; metadata declares mm and 100% real-world files, recommends 125% teaching prints; CC-BY-NC-SA; individual L4/L5, hips, sacrum and proximal-femur files | Documented artifact cleanup and sculpted restoration prevent automatic metric admission. Units/scale declaration is retained, but landmark displacement and original-imaging comparison remain unresolved. Proximal femur is not a complete femur. |

PHY-REF remains the intended home for original/reference geometry. No unqualified
mesh or large imaging dataset was mirrored into PHY or PHY-REF. The two teaching
entries' restricted licenses remain source attributes; repository Unlicense
does not override third-party asset terms. This PR redistributes factual table
extracts and original engineering descriptions, not source articles, images or
NIH mesh derivatives.

## Derivation, uncertainty and maturity

`references/dim072/catalog.json` is the admission ledger. Seven selected primary
morphometry studies supply the new values; several are hosted in NIH's PMC
archive, which does not make them NIH-authored standards. Each source records
the publication/revision, population subset, units, measurement axes, rights
disposition, inspected method/table, tool version, uncertainty and limitations.
Native facts are retained in separate hashed snapshots, including rejected
measurements. Each canonical field has a provenance pointer to its reviewed
native row and explicit dimensional meaning.

All derivations in this slice use the published mean in millimeters unchanged.
No stature scaling, point registration, mesh bounding-box measurement or native
left/right mirroring is performed. Published SD is preserved as variability, not
treated as a measurement-error bound, allowable tolerance or individual fit
guarantee. No direct registration to SOPHY 1676.4 mm has been established.

Two native entries are deliberately rejected: C2 HT pending confirmation of
odontoid/span semantics, and T11 posterior body height because its table value
conflicts with the narrative maximum and resembles the chord-length row. Neither
is corrected from neighboring levels. C1 has no centrum; no C1 body height is
invented. Canal AP/transverse spans remain void measurements, not bone width or
thickness.

| Maturity | Meaning in the delivered audit |
| --- | --- |
| Declared | Every currently declared field is finite, positive and non-null. The unchanged audit counts 40 maps. |
| Source-backed | All new non-null values have reviewed numerical lineage. Four newly complete maps and 28 partial maps satisfy this limited scope. Existing 36 are not promoted. |
| Geometry-qualified | Surface, landmarks, registration and shape fidelity independently reviewed. Zero additions qualify. |
| Fabrication-qualified | Accepted interfaces, tolerances, materials, inspection and physical evidence. Zero additions qualify. |

Mass remains explicitly unknown for changed records. Existing material/tolerance
software defaults are not admitted physical data. Geometry, joints and mounts
remain unresolved. Complete dimensions do not make a validated fabrication
export or a qualified anatomical surface.

## Reproduce and review

```bash
python3 bin/audit_engineering.py --dataset female_21_baseline --output-root /tmp/phy-engineering
python3 bin/audit_dimension_expansion.py --output-root /tmp/phy-dim072
python3 bin/audit_dimension_expansion.py --output-root /tmp/phy-dim072 --require-target
```

The expansion CLI works outside the checkout using only the standard library.
Exit 0 means the evidence comparison report was produced; `target_met` remains
false. With `--require-target`, this slice exits 2 for its honest shortfall.
Evidence errors exit 1 and do not overwrite a prior successful report package.
Reports are deterministic and have a final manifest; readers verify its hashes.

The checker refuses duplicate IDs/JSON keys, unsupported numbers, loss of a
previous field or unknown, missing provenance, altered native facts, semantic
relabeling, missing side assumptions, unknown units, unreviewed scale/coordinate
changes, parent cycles and inferred physical qualification. An intact hash or
successful parse is only an integrity observation, never source admission.

Focused evidence tests and existing engineering/export/Studio/canon checks are
reported in `reports/DIM072_VALIDATION.md`. The Studio reference proxies remain
independent authored geometry: new reference statistics are not silently used to
resize them. Physical fit and whole-body interference remain unresolved.

## Fabrication sequence after this slice

1. Admit a segmented female pelvis/lower-lumbar source with verified metric
   fidelity and usable rights; define L5/sacrum/hip landmark datums and a
   whole-vertebra dimensional contract.
2. Develop an adjacent lumbar cartridge with neutral/end-stop transforms,
   stiffness/load basis, bearing/stop interfaces, mounting tolerances and an
   inspection fixture. Do not sum marginal body heights into an assembled spine.
3. Define tibia/talus/calcaneus contact datums and full hindfoot/navicular
   geometry. The new spans support comparison; they do not specify an ankle
   bearing, contact surface or support polygon.
4. Resolve shoulder/thorax roots and the existing A0 bearing/stop release
   blockers on their own physical-evidence track.

Retain the branch and review the PR. The target remains 72; improving the count
by dropping unknowns or relabeling partial anatomy is prohibited.
