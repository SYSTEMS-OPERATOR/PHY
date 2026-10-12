# Craniofacial armature and external-feature overlap

Base: PR #107 merge `da666467d30cf8bb25b66ac38951369bc5a02ac8`.
Branch: `agent/phy-craniofacial-relations`.

PHY's subject remains the internal armature. External features impose packaging,
support and clearance questions at the boundary, but a surface preference does
not define the underlying bone dimensions or a mechanical joint. This increment
adds a qualitative relationship catalog and an offline regional evidence audit.
It supplies no new geometry or adopted facial ratios.

## Selected scope and evidence

`references/craniofacial_relationships.json` defines seven selected regions:
right/left orbit, nasal bridge, bony nasal septum, hard palate, the single adult
mandibular unit and the paired zygomatic arches. Seventeen existing skull
identities participate. Membership states a structural contribution; it does
not create pairwise adjacency among every member, identify a suture, or assign
a parent. This is not a complete cranial contact graph.

The catalog separately defines the right/left TMJ articulations, sharing the
single `BONE_MANDIBLE`. OpenStax's joint reference describes both rotation and
translation, with an intervening disc. Thus an anatomical participation map
cannot justify a fixed joint or claim equivalence for a single-axis jaw hinge.
Both articulation engineering definitions remain null. Trajectories, axes,
ranges, interfaces, substitute disc geometry, loads and acceptance evidence
are required before mechanism qualification.

Sources reviewed on 2026-10-12:

- [OpenStax A&P 2e, 7.2 The Skull](https://openstax.org/books/anatomy-and-physiology-2e/pages/7-2-the-skull): selected structural contributions.
- [OpenStax A&P 2e, 9.6 Anatomy of Selected Synovial Joints](https://openstax.org/books/anatomy-and-physiology-2e/pages/9-6-anatomy-of-selected-synovial-joints): TMJ participants and qualitative motion.
- [Pallett, Link and Lee (2010), PMID 19896961](https://pubmed.ncbi.nlm.nih.gov/19896961/): indexed abstract reviewed as adjacent perception context only.

The third source experimentally compares altered facial feature spacing. It
does not supply an adopted craniofacial bone model, orbital center, eye mount or
bone-to-surface offset here. No numerical finding is adopted. It was located
anew during this investigation; the original paper citations and datasets behind
the 2025 facial study discussions have not been recovered. Historical ratio
proposals therefore remain research context, not qualified source measurements.

Each source declares its reviewed scope. The catalog checker prevents a
perception-context reference from supporting an anatomical claim. It also
rejects unknown or non-skull participants, opposite-side orbital participants,
duplicate region/joint identities, reduced TMJ motion, unsupported fields and
non-null spatial/mechanical definitions. These are contract checks on curated
declarations, not automatic verification of a publisher's text.

## Boundary review

For every region, the report joins its participating bone IDs to the existing
engineering audit's source paths, canonical records, declared dimensions,
dataset bindings, geometry status, legacy relationship issues and Studio mesh
IDs. This makes an orbit or palate inspectable as a multi-bone structure without
editing the viewer or treating its provisional mesh as anatomy.

`external_overlap` and `evidence_needed` entries are engineering review questions.
They identify missing eye-module envelopes, eyelid/lip swept clearances, nasal
cartilage or shell layouts, oral/nasal separation, occlusal datums and surface
offsets. Their presence is not evidence that the feature fits, that those
surfaces have been defined, or that a material/mechanism has been selected.
All seven spatial definitions remain null; zero articulations are mechanically
defined. The audit retains fabrication release false and facial canon unbound.

The current F28 head guide is included as a separately labeled observation:

| Guide axis | Existing basis | What it does not establish |
| --- | --- | --- |
| Subject-right breadth | Pinned ANSUR female age-28 marginal headbreadth mean | Orbital separation, bony breadth, or cheek surface |
| Anterior depth | Pinned ANSUR marginal headlength mean | Nasal projection, facial depth, or skull surface |
| Superior height | Project's existing 220 mm head-height choice | A measured head height or an adopted SOPHY ratio |

The cohort contains 92 measured US Army women aged 28. Its marginal means are
not a real individual or a representative worldwide sample. All three inputs
undergo the existing global stature scale; that transform is display design,
not a new anthropometric observation. The CLI defaults to a 1676.4 mm stature
inspection instance. It does not change the F28 profile or SOPHY canon 1.0.0.

## Run and inspect offline

```bash
python bin/audit_craniofacial.py --output-root /tmp/phy-head-review
python -m unittest discover -s tests -p test_craniofacial_audit.py -q
```

The command can run outside the checkout and requires no network, CAD package
or browser. It writes `reports/craniofacial_audit.json`, a readable Markdown
report and a manifest using the existing rollback publisher. The manifest pins
the canonical records, display parts, datasets, canonical JSON catalog
fingerprint and head profile, and hashes both report files. Missing canonical
records/proxies are reportable gaps. Malformed declarations or invalid model
inputs reject before publication and preserve the previous package.

Use the report's Studio mesh IDs to inspect the existing proxies. Detailed
source coverage and legacy relationship blockers are in the JSON; an anatomical
TMJ declaration does not repair the legacy mandible's ambiguous `temporal`
parent/child references. No BoneSpec connections or joint interfaces are filled.

## Next evidence

Select an orbital housing or mandibular article as a bounded next study. Obtain
traceable surfaces/landmarks and actual module packaging requirements, then
compare proposed interfaces and swept clearance with the external envelope.
Review any facial coordinate or ratio adoption as a separate canon decision.
The relationship map can mature independently of those measurements.
