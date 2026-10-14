# PHY-RECOVER-072 — recovered checkpoint

Recovery began 14 October 2026 (UTC). Repository reads, local working trees and
content hashes take precedence over the interrupted activity log. No implementation
was restarted before this inventory. Full historical Work command history is not
available; surviving files, visible conversation context and GitHub objects are.

## Durable state at recovery

| Repository | Verified revision | Commit time (UTC) | State |
| --- | --- | --- | --- |
| PHY main | `2064af478d6554f66900f655afdd64c8869a16b5` | 2026-10-12T20:06:20Z | Unchanged since checkpoint; 36 complete declared maps |
| PHY dimension branch | `d8afe81b75c09719cbfe7c8f001b1075590c493c` | 2026-10-13T20:33:37Z | One commit ahead of main; 40 complete maps, 93 new fields; no PR/check run at recovery |
| PHY-REF main | `5d4cd1e841ec275580e6634fee760e9615f02c30` | 2026-10-12T18:57:42Z | Reference acquisition register only |
| PHY-REF interrupted tree | `0e5de1cc04f17ee6d0d7e300f68bb391f13604ce` | Tree objects have no commit timestamp | Uploaded Git tree, matching local index; no reachable commit/ref at recovery |
| PHY-REF- main | `256edbd55ad00b3abcf78e6b072387b2f490fb6a` | 2026-10-12T18:22:30Z | LICENSE and 21-byte README only; no physics implementation |

The owner repository inventory and creation-date search found no other new
physics-reference repository. `PHY-REF-` was created at 18:05:00Z on October 12;
PHY-REF at 18:14:47Z. The former has no evidenced engineering responsibility.
No new repository was created, renamed or deleted by this recovery.

## Recovery ledger

| Work item | Classification at recovery | Evidence | Outstanding validation |
| --- | --- | --- | --- |
| NIH reference acquisition | Complete locally; publication partial | Three pinned originals, independent download copies and LFS objects; catalog in PHY-REF | Verify remote LFS availability |
| Source provenance records | Complete for byte custody; partial for metric qualification | `sources/reference_assets.json`, `NIH_entry_evidence.json`, attribution and CC BY 4.0 text | Donor-scale fidelity, expert landmark/segmentation review |
| GLB parser and validation | Complete within declared static-parser scope | `phy_ref/glb.py`, cached audits, 11 tests | Fresh CI, no claim of manifold/anatomical certification |
| Native-frame comparison GLBs | Complete | Pelvis/skin previews, source/tool/output hashes and sampled-error manifest; previous repeated outputs match | Visual inspection; not geometry qualification |
| Physics reference repository | Missing implementation | Owner inventory; PHY-REF- contains only initial files | No independent repository justified for this slice |
| Dimension-map expansion | Partial | October 13 commit, seven hashed fact snapshots, admission ledger and reports | Reverify sources/software; 40/72 complete, 28 improved partial maps |
| Tests and CI | Prior results recorded; fresh local reference tests passed; no branch CI at recovery | `reports/DIM072_VALIDATION.md`; 11 reference tests rerun | Fresh CI and exact recovery validation report |
| PR and merge status | No existing PRs; neither branch merged | GitHub main/branch/PR reads | Recovery PRs prepared below |

One body working copy was truncated to 87,117,312 bytes, although its GLB header
declared 211,624,968. Its SHA-256 was
`2cd61a76dfedd4dc9cccc5ebc9455dd9368fce21f08b38bbe397fd8014fcc2c6`.
The complete acquisition copy and local LFS object independently matched the
pinned SHA-256 `472567a56896b9b7890508da6501fbf858e56aaa30745365f7a71ade782b529c`.
The damaged copy was retained separately and the working file restored from the
verified copy. No source was redownloaded and no valid binary was regenerated.

Four uncommitted PHY research drafts also survived. Their exact paths, sizes and
hashes are recorded in `reports/recovery/surviving_drafts.json`. They were not
mistaken for dimension implementation or accepted canonical transforms.

## Architecture and authority

PHY-REF owns original NIH assets, immutable byte-custody records, native derived
comparison geometry and reference qualification evidence. PHY owns canonical
identities/dimension declarations, article interfaces and simulation decisions.
NASA PDFs remain in PHY under the existing archival convention. Cross-repository
consumers must pin both a commit and the relevant asset hash. Entry versions,
HRA object versions and byte hashes are distinct identifiers.

Originals remain native GLB geometry. Reduced previews are separately identified;
no transformed mesh becomes an original. Canonical registration remains null.
Mesh labels, extrema and a successful parser do not establish osteometric
landmarks. The lumbar-6 label remains unmapped; no canonical renumbering occurs.

The recovered dimension branch uses seven primary morphometry publications,
including papers archived by NIH PMC. Archive hosting does not make these NIH
standards. The 36 historical complete maps remain legacy declarations. No tissue
model, physics repository, NASA compliance or physical release is introduced.

## Review destinations

- [PHY PR 110](https://github.com/SYSTEMS-OPERATOR/PHY/pull/110): recovered dimension evidence, further verification and bounded reference reconciliation.
- [PHY-REF PR 1](https://github.com/SYSTEMS-OPERATOR/PHY-REF/pull/1): recovered acquisition, originals, parser and native previews.

The original `phy-dim-072-evidence` branch is preserved. Recovery PRs use separate
branches and remain unmerged. Acceptance depends on fresh validation, explicit
shortfall reporting and review of the supported evidence scope.

## Continuation result

The 93 recovered source-backed fields remain unchanged. Independent column
extraction rechecked all 126 current native mean/SD rows across seven primary
publications. Recovery added 35 explicitly named thoracic superior/middle/inferior
body widths. T6 superior width is retained but withheld because the printed
aggregate means disagree with cohort weighting. C2 HT and T11 posterior height
remain rejected. No substitute measurement is manufactured.

The result is **40/206 complete declared maps, 28 improved partial maps and 128
new source-backed fields** relative to main. Four complete additions are
BONE_PATELLA_L, BONE_PATELLA_R, BONE_TALUS_L and BONE_TALUS_R. The 72 target
remains short by 32 maps. All 206 identities, the previous 36 complete maps and
all recovered declarations are retained. Geometry- and fabrication-qualified
additions remain zero; full record validation still passes 0/206.

Recovered source-label correspondence is now checked against an immutable
PHY-REF audit: 36 canonical labels / 46 mesh nodes, with lumbar-6 unmapped.
The frame fitter consumes explicit named landmarks and units, performs only a
proper rigid fit and reports reference consistency. Synthetic positive/negative
tests establish software behavior; no real anatomical packet has been accepted.
A proposed view rotation is not a canonical transform.

NASA review preserves 52 condition-specific motion pairs and 11 anthropometric
comparison ranges in references/nasa_positioning_ranges.json. Source signs,
posture/cohort scope and ambiguous labels are retained. The neck table/diagram
conflict and 16 pressurized-table heading ambiguities remain flagged. Mechanical
axes, limits and combined-joint feasibility are null. OCHMO-HB-004 Revision A is
the current positional/methodological reference for this slice; archived HIDH
Section 4 is historical and carries a 2022 replacement notice. None supplies
invented osteometry or evidence of PHY zero-G capability.

## Gap and dependency matrix

| Constraint | Verified gap | Dependency / next evidence | Release consequence |
| --- | --- | --- | --- |
| Whole vertebrae | All C/T/L generic whole-bone extents remain unknown; body/endplate spans are partial | Define osteometric contract, posterior-element landmarks and source metric fidelity | No full-spine fabrication inference from populated fields |
| Pelvis/sacrum | Native segmented references survive; current canonical maps are legacy declarations | Expert sacral/hip datums, bilateral asymmetry, contact surfaces, error budget | Hold pelvis/spine/hip placement and mounts |
| Lumbar variant | HRA source labels L1–L6 | Review numbering and transitional anatomy independently | Do not assume canonical L5–sacrum adjacency in this donor |
| Lower limbs | Long-bone declarations exist; source meshes available | Head/condyle/plateau/ankle datums and tissue exclusions | Bone envelope does not locate a bearing or contact axis |
| Hindfoot | Talus/patella complete comparison spans; calcaneus partial; navicular unresolved | Qualified whole surfaces, ankle/subtalar and support datums | No ankle or foot fabrication release |
| Skull/ribs/hands/feet | HRA assembly lacks complete coverage; other references remain scoped | Small, rights-usable segmented sources or complete osteometry | Count cannot reach 72 defensibly with current qualified evidence |
| Kinematics / Studio | 206 proxies retained; display dimensions independent; no adopted mesh registration | Datum/axis convention, explicit joint constraints and interference validation | Human ROM tables cannot act as mechanical stops |
| Shoulder A0 | Existing bearing and stop evidence blockers persist | Independent accepted load/bearing/stop tests | No shoulder redesign or release in recovery |
| Mass / interfaces | 206 records still lack qualified mounts and joint interfaces | Article-specific load basis, materials, tolerances, physical inspection | Source/software success does not imply fabrication readiness |

## Ranked execution paths

Effort estimates are planning judgments, not vendor quotes or committed schedules.

| Rank / path | Evidence and prerequisites | Effort / compute | Tradeoff and stop/go criterion |
| --- | --- | --- | --- |
| 1 — minimum-risk consolidation | Review these two PRs; preserve pinned custody, counts and physical unknowns | Roughly 1–2 engineering days for review; CPU-only audits around tens of seconds; no new asset downloads | Locks down evidence without claiming mechanical progress. Go when CI, hashes, rights and baseline preservation are accepted; stop on source drift or ambiguous dimensional semantics. |
| 2 — highest-value advancement | Native sacrum + one fused hemipelvis datum packet, expert scale/segmentation review, defined article coordinate conventions | Roughly 3–5 engineering days plus reviewer availability; bounded mesh subset and CPU landmark work | Advances the load path toward a reviewable fixture. Go only after independent landmarks, residual/error budget and variant disposition; stop before geometry adoption if fidelity remains unknown. |
| 3 — exploratory research | Additional complete female osteometry/segmentation for posterior vertebrae and hindfoot; choose one unresolved region | Time-box about 2 days initially; small tables/meshes only, explicit budgets | May increase defensible coverage. Stop if rights, units, anatomical scope or reproducibility cannot be established; do not import a large imaging collection to chase the count. |

A generic deformable-body runtime is **later**. No qualified mass, attachment,
elasticity or damping packet currently supports it, and it does not close the
current datum/interface blockers. If a later article needs compliance or coupled
motion, begin with a reduced-order model tied to measurable attachment/load
observables. Zero-G mechanics changes external body-force conditions; it does
not remove inertia, establish biological response or validate a visual shape.
No soft-tissue implementation or new repository belongs in this recovery.

## Recommended bounded next slice

Prepare a **native sacrum–hemipelvis datum review packet**, using original
NIH_3DPX_020984 and one explicitly selected side. Preserve the unmodified original
and pin every annotation to its source hash, node and vertex/face location.
Define sacral-base/endplate and iliac/acetabular landmarks, with native m units,
explicit definitions, annotation method and uncertainty. Keep unreviewed points
null; defer hardware centers and transformed production geometry.

Expected files: PHY-REF sources/landmarks/pelvis_review.json and a small labelled
review view; PHY references/landmarks/pelvis_crosswalk.json, one deterministic
reference comparison report, and focused custody/fit tests. These are proposed
paths, not a parallel canonical geometry authority.

Acceptance: verified scale/segmentation evidence; independently repeated landmark
annotations; explicit residual/error budget approved for the intended article;
variant and side disposition recorded; native/derived lineage preserved; 206
identities and SOPHY 1.0.0 1676.4 mm height/span unchanged. Positive tests must
reproduce a known fit; negative tests must reject missing/mirrored/scaled or
ambiguous correspondence and altered source bytes. Deliver a datum packet and
fixture inspection proposal for review, not a full pelvis mechanism or released
manufacturing drawing.

## Decisions requiring SYSOP review

- Accept the evidence scope and explicit 40/72 shortfall in the recovery PRs.
- Define whole-bone measurement contracts for irregular bones without weakening
  the completeness counter or substituting endplate/void spans.
- Resolve donor-scale fidelity and lumbar-6 disposition before PHY adoption.
- Select the first pelvis side/article, expert reviewer and admissible annotation
  uncertainty; approve actual datums and fit thresholds when the packet exists.
- Decide which NASA human-interface references become article requirements;
  preserve separate mechanical ranges, loads and zero-G scenarios.
- Maintain A0 bearing/stop release as its own evidence track.

Validation commands, failures, skips and repeatability are recorded in
reports/RECOVERY_VALIDATION.md. The complete dimension matrix and source rows are
in reports/dimension_expansion.md and reports/recovery/source_reverification.json.
