# Spine dimensions and feature review

This pass reviews all 26 adult spine identities: C1-C7, T1-T12, L1-L5,
one sacrum and one coccyx. BoneSpec remains authoritative. Qualitative anatomy,
tabular measurements, display geometry and engineering qualification are separate.
No canon, anatomical coordinate, mechanical limit or model mesh is changed.

## Results and corrections

| Finding | Disposition | Regression coverage |
| --- | --- | --- |
| The 24 unfused vertebrae have 102 admitted body/canal spans, not complete whole-bone envelopes. | Checked the published tables; every admitted number is retained exactly. Whole-bone length/width/thickness remain null. | Every one of the 206 dimension maps must equal the pinned DIM-072 baseline plus its admitted additions. |
| C1 declared `skull`, which exact identity resolution cannot resolve. | Declare `BONE_OCCIPITAL`, the bone bearing the occipital condyles. | Fresh field/export resolution; expected superior anatomical references for all 26 identities. |
| Coccyx declared lower-case `sacrum`, which exact export/render matching cannot resolve. | Declare `BONE_SACRUM`. Matching policies are unchanged. | Exact identity resolution and full superior-reference chain. |
| Every spine `notable_features` list was empty. | Add cited qualitative declarations, distinguishing atlas/axis, thoracic facet exceptions and fusion variability. | BoneSpec declarations checked against the bounded source review; missing/altered references and features are diagnosed. |
| GeometryAgent read only `_cm` keys. A 100 x 110 x 10 mm irregular record produced no runtime primitive while an equivalent cm record did. | Reuse BoneSpec's normalization/alias checks and convert canonical mm to runtime meters. Reject nonpositive/nonfinite primitive sizes. | Equivalent cm/mm/m/mixed-unit inputs; alias conflicts; invalid values; all real unfused vertebrae and coccyx still produce no solid envelope from feature spans. |
| Canal spans were described as clearances. | Clarify that osseous opening spans are not usable mechanical clearances, solid-bone dimensions or cartridge envelopes. | Body/canal spans never substitute for whole-bone dimensions. |

Anatomical parent references are a declared superior-reference hierarchy. They
do not define motion axes, intervertebral disc geometry, facet contact, a rigid
mechanical tree or signal routing. Source articulations and runtime links are
unchanged. In particular, these changes do not turn the atlas-axis complex into
a single engineered hinge.

## Numerical source review

The review starts from main `fa02634120438687d67c6d156aa0aec1adf41aa1`.
[The source-review record](../references/spine/source_review.json) records fresh
response hashes, source URLs, local fact hashes, columns and comparison methods.
Source articles and images are not redistributed.

| Source | Printed rows checked | Admitted rows retained | Method |
| --- | ---: | ---: | --- |
| [Ekizoglu et al., cervical CT](https://pmc.ncbi.nlm.nih.gov/articles/PMC8354927/) | 20 | 19 | XML Table 2 female training columns; mean, SD, minimum and maximum; validation columns excluded. |
| [Singh et al., thoracic calipers](https://www.asianspinejournal.org/upload/pdf/asj-5-20.pdf) | 60 | 58 | Publisher PDF page 7 / printed page 26, Table 3 female columns; text extraction plus rendered visual review. |
| [Kot et al., lumbar CT](https://pmc.ncbi.nlm.nih.gov/articles/PMC8917002/) | 25 | 25 | XML Table 1 female upper/lower body margins; means/SD; disc rows excluded. |

There were **no numerical transcription mismatches across 105 native rows**.
That does not establish donor-specific dimensions or source-study accuracy.
C2HT, T6_VBWs and T11_VBHp remain quarantined, with their printed values preserved.
No replacement value or stature-based scaling is introduced. Published SD remains
cohort variation, not manufacturing tolerance or a measurement-error bound.

The cervical paper's Results give 106 male and 104 female developmental subjects.
The same paragraph calls the 40 male / 44 female remainder a training set,
whereas the Methods and Table 2 separate training and validation. This wording
conflict is recorded; rows from the two table sections are not pooled.

The default dataset supplies sacrum spans of **100 x 110 x 10 mm** and produces
an estimated mass. These are legacy, unverified inputs, not measurements admitted
by the three vertebral studies. They are retained and explicitly labeled, not
promoted to anatomical truth. Coccyx quantitative dimensions remain unknown;
published straight and curved lengths cannot silently become one generic length.

## Qualitative features and geometric limits

[The feature review](../references/spine/features.json) cites the publisher's
[OpenStax Anatomy and Physiology 2e section 7.3](https://openstax.org/books/anatomy-and-physiology-2e/pages/7-3-the-vertebral-column),
[the thoracic reference](https://www.ncbi.nlm.nih.gov/books/NBK459153/) and
[Woon et al.'s coccyx CT study](https://pmc.ncbi.nlm.nih.gov/articles/PMC3631051/).
It is qualitative reference evidence, not another canonical bone or geometry model.

- C1 is an arch/ring structure without a body or spinous process; C2 has the dens.
- Typical cervical foramina and commonly bifid processes are distinguished from
  C7's prominent process. Variability is not replaced by a universal shape.
- T1's costal facets differ from typical thoracic demifacets. T9/T10 patterns can
  vary. T11/T12 lack transverse costal facets; T12 is transitional.
- Lumbar body, neural-arch and process features remain separate from the admitted
  body/endplate measurements. Qualitative facet orientation is not a joint axis.
- Sacrum and coccyx each retain one adult inventory identity. Coccygeal segment
  number and fusion are variable, not a mandatory four-piece or fully fused model.

Feature dimensions and mechanical definitions remain null. Existing Studio spine
ellipsoids are **unverified display proxies**: they do not model canal openings,
pedicles, laminae, dens, facets, processes, sacral foramina or sacral hiatus.
No proxy, facet orientation or feature description is used as numerical anatomy.

## Offline inspection and verification

```sh
python bin/audit_spine.py --output-root /tmp/phy-spine --require-consistent
python -m unittest tests.test_geometry_units tests.test_spine_audit -q
```

The CLI publishes JSON, a Markdown dimension/feature inspection report and a hash
manifest using existing staging/rollback utilities. Inputs are fresh BoneSpec
records, current source files and pinned source facts, not cached audit reports.
The standard-library API is `audit_spine(field)`; partial inputs retain missing
records explicitly. Construction does not change source objects or runtime state.

Exit 0 means a report was produced; `--require-consistent` returns 2 for data
inconsistencies. Input/publication errors return 1. A consistency pass never means
that all dimensions are verified, anatomical surfaces are qualified, physical
evidence is measured or fabrication is released. Review report/manifests must be
verified before use. The JSON retains all dimensions, source locators and hashes.

The ordinary fabrication package/schema is unchanged. Features remain descriptive
BoneSpec data and are included with their citations in the spine audit. The
normal review package refreshes the corrected anatomical references and citations.
Studio's existing source baseline must be explicitly reviewed and repinned to
these source changes; its stale-evidence gate remains enabled.

SOPHY canon 1.0.0 and the active T56 H = span = 1676.4 mm remain exact. Scale,
materials and mechanisms may change; canon does not drift. Anatomy still needs
qualified feature surfaces, registration, interfaces and physical evidence before
it can support a fabrication release.
