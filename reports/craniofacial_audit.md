# Craniofacial armature boundary review

Qualitative anatomy and evidence gaps; fabrication release remains false.

Region participation is not pairwise adjacency, an assembly parent or a mechanical interface.

| Observation | Count |
| --- | ---: |
| structural_regions | 7 |
| qualitative_articulations | 2 |
| participating_bone_identities | 17 |
| canonical_records_present | 17 |
| individual_display_proxies | 17 |
| spatially_defined_regions | 0 |
| mechanically_defined_articulations | 0 |

## Structural regions

External overlap and evidence-needed entries are engineering review questions; they do not qualify feature fit.

### Hard palate and upper oral boundary

Source: `OPENSTAX_AP2E_7_2_CRANIOFACIAL` / Maxillary Bone; Palatine Bone; Figure 7.8.

| Canonical bone ID | Anatomical contribution | Studio mesh IDs |
| --- | --- | --- |
| BONE_MAXILLA_L | anterior palate; upper jaw and tooth support | maxilla_L |
| BONE_MAXILLA_R | anterior palate; upper jaw and tooth support | maxilla_R |
| BONE_PALATINE_L | posterior palate | palatine_L |
| BONE_PALATINE_R | posterior palate | palatine_R |

Overlap to review: Oral and nasal cavity separation; upper dentition and mouth envelope.

Evidence needed: Palatal and maxillary surfaces; Oral and nasal envelopes; Dentition or mechanical substitute and occlusal datum.

Spatial definition: unresolved. Missing canonical records: none; missing display proxies: none.

### Single adult mandibular unit

Source: `OPENSTAX_AP2E_7_2_CRANIOFACIAL` / Mandible; Figure 7.15.

| Canonical bone ID | Anatomical contribution | Studio mesh IDs |
| --- | --- | --- |
| BONE_MANDIBLE | lower jaw, lower tooth support and bony chin | mandible |

Overlap to review: Moving lower mouth envelope and chin support; lip and chin surfaces remain separate.

Evidence needed: Mandibular surfaces and landmarks; Coupled bilateral TMJ trajectory; Occlusal geometry, chin offsets and lip swept clearance.

Spatial definition: unresolved. Missing canonical records: none; missing display proxies: none.

### Bony nasal bridge

Source: `OPENSTAX_AP2E_7_2_CRANIOFACIAL` / Nasal Bone.

| Canonical bone ID | Anatomical contribution | Studio mesh IDs |
| --- | --- | --- |
| BONE_NASAL_L | paired bridge and cartilage support | nasal_L |
| BONE_NASAL_R | paired bridge and cartilage support | nasal_R |

Overlap to review: Nasal shell support; external tip and alar shape require separate definition.

Evidence needed: Nasal bone surfaces and landmarks; Cartilage or shell layout; Bone to external surface offsets.

Spatial definition: unresolved. Missing canonical records: none; missing display proxies: none.

### Bony nasal septum

Source: `OPENSTAX_AP2E_7_2_CRANIOFACIAL` / The Nasal Septum and Nasal Conchae; Figure 7.17.

| Canonical bone ID | Anatomical contribution | Studio mesh IDs |
| --- | --- | --- |
| BONE_ETHMOID | upper bony septum | ethmoid |
| BONE_VOMER | posterior and inferior bony septum | vomer |

Overlap to review: Nasal chamber separation; anterior septal cartilage remains outside this bone map.

Evidence needed: Septal surfaces; Nasal chamber clearance requirements; Anterior cartilage or substitute geometry.

Spatial definition: unresolved. Missing canonical records: none; missing display proxies: none.

### Left bony orbit

Source: `OPENSTAX_AP2E_7_2_CRANIOFACIAL` / The Orbit; Figure 7.16.

| Canonical bone ID | Anatomical contribution | Studio mesh IDs |
| --- | --- | --- |
| BONE_ETHMOID | medial wall | ethmoid |
| BONE_FRONTAL | roof | frontal |
| BONE_LACRIMAL_L | medial wall | lacrimal_L |
| BONE_MAXILLA_L | medial floor | maxilla_L |
| BONE_PALATINE_L | small floor contribution | palatine_L |
| BONE_SPHENOID | posterior orbit | sphenoid |
| BONE_ZYGOMATIC_L | lateral wall and lateral floor | zygomatic_L |

Overlap to review: Left eye module housing and eyelid clearance.

Evidence needed: Orbital surfaces and landmarks; Eye module envelope and mounting requirements; Soft tissue offsets and eyelid swept clearance.

Spatial definition: unresolved. Missing canonical records: none; missing display proxies: none.

### Right bony orbit

Source: `OPENSTAX_AP2E_7_2_CRANIOFACIAL` / The Orbit; Figure 7.16.

| Canonical bone ID | Anatomical contribution | Studio mesh IDs |
| --- | --- | --- |
| BONE_ETHMOID | medial wall | ethmoid |
| BONE_FRONTAL | roof | frontal |
| BONE_LACRIMAL_R | medial wall | lacrimal_R |
| BONE_MAXILLA_R | medial floor | maxilla_R |
| BONE_PALATINE_R | small floor contribution | palatine_R |
| BONE_SPHENOID | posterior orbit | sphenoid |
| BONE_ZYGOMATIC_R | lateral wall and lateral floor | zygomatic_R |

Overlap to review: Right eye module housing and eyelid clearance.

Evidence needed: Orbital surfaces and landmarks; Eye module envelope and mounting requirements; Soft tissue offsets and eyelid swept clearance.

Spatial definition: unresolved. Missing canonical records: none; missing display proxies: none.

### Paired zygomatic arches

Source: `OPENSTAX_AP2E_7_2_CRANIOFACIAL` / Temporal Bone; Zygomatic Bone; Figure 7.5.

| Canonical bone ID | Anatomical contribution | Studio mesh IDs |
| --- | --- | --- |
| BONE_TEMP_L | posterior left arch | temp_L |
| BONE_TEMP_R | posterior right arch | temp_R |
| BONE_ZYGOMATIC_L | anterior left arch | zygomatic_L |
| BONE_ZYGOMATIC_R | anterior right arch | zygomatic_R |

Overlap to review: Lateral cheek envelope and jaw mechanism packaging.

Evidence needed: Arch surfaces and landmarks; External cheek offsets; Jaw mechanism and cable swept clearances.

Spatial definition: unresolved. Missing canonical records: none; missing display proxies: none.

## Mandibular system

Both TMJ sites share `BONE_MANDIBLE`; two sites do not create two independently positioned jaws.

| Articulation | Bone IDs | Anatomical motion | Engineering definition |
| --- | --- | --- | --- |
| tmj_l | BONE_MANDIBLE, BONE_TEMP_L | rotation, translation; articular disc | unresolved |
| tmj_r | BONE_MANDIBLE, BONE_TEMP_R | rotation, translation; articular disc | unresolved |

Anatomical motion includes rotation and translation. Axes, trajectory, ranges, disc/substitute surfaces and mechanical acceptance remain unknown.

## Existing head guide inputs

Measured female US Army personnel, 2010–2012. Not a representative civilian or worldwide sample.

Global stature scaling is a display design transform. Marginal means are not a measured individual or facial landmark map.

| Axis | Input | Basis | Guide value (mm) |
| --- | --- | --- | ---: |
| x | headbreadth | external_cohort_marginal_mean | 151.933203 |
| y | headlength | external_cohort_marginal_mean | 195.221497 |
| z | head_height | project_design_choice | 225.929618 |

## Canonical and display evidence

| Bone ID | Canonical record | Declared dimensions | Canonical geometry | Existing record gate |
| --- | --- | --- | --- | --- |
| BONE_ETHMOID | present | complete | dimensional_estimate | blocked |
| BONE_FRONTAL | present | complete | dimensional_estimate | blocked |
| BONE_LACRIMAL_L | present | unknown | unresolved | blocked |
| BONE_LACRIMAL_R | present | unknown | unresolved | blocked |
| BONE_MANDIBLE | present | complete | dimensional_estimate | blocked |
| BONE_MAXILLA_L | present | unknown | unresolved | blocked |
| BONE_MAXILLA_R | present | unknown | unresolved | blocked |
| BONE_NASAL_L | present | unknown | unresolved | blocked |
| BONE_NASAL_R | present | unknown | unresolved | blocked |
| BONE_PALATINE_L | present | unknown | unresolved | blocked |
| BONE_PALATINE_R | present | unknown | unresolved | blocked |
| BONE_SPHENOID | present | complete | dimensional_estimate | blocked |
| BONE_TEMP_L | present | complete | dimensional_estimate | blocked |
| BONE_TEMP_R | present | complete | dimensional_estimate | blocked |
| BONE_VOMER | present | unknown | unresolved | blocked |
| BONE_ZYGOMATIC_L | present | unknown | unresolved | blocked |
| BONE_ZYGOMATIC_R | present | unknown | unresolved | blocked |

## Reference scope

| Source | Kind | Locator / reviewed scope |
| --- | --- | --- |
| [OPENSTAX_AP2E_7_2_CRANIOFACIAL](https://openstax.org/books/anatomy-and-physiology-2e/pages/7-2-the-skull) | anatomical_reference | The Orbit; Nasal Bone; The Nasal Septum and Nasal Conchae; Maxillary Bone; Palatine Bone; Mandible; Temporal Bone; Zygomatic Bone. Qualitative structures only. |
| [OPENSTAX_AP2E_9_6_TMJ](https://openstax.org/books/anatomy-and-physiology-2e/pages/9-6-anatomy-of-selected-synovial-joints) | anatomical_reference | Temporomandibular Joint. Articular participants, disc, and combined hinge/gliding motion; no adopted axes, ranges or interface geometry. |
| [PALLETT_LINK_LEE_2010_PERCEPTION](https://pubmed.ncbi.nlm.nih.gov/19896961/) | perception_context | Indexed abstract only: paired attractiveness comparisons with altered eye-mouth and interocular spacing. No ratios or population means adopted; not recovered proof of the 2025 study citations. |

## Limits

- Selected structures only; not a complete skull contact or suture graph.
- Region participation does not imply pairwise contact, a rigid parent or a mount.
- Anatomical TMJ declarations do not resolve legacy temporal references or qualify mechanisms.
- External head means and a project head-height choice do not locate orbital centers or define bone-to-skin offsets.
- Perception research is adjacent design context; it does not qualify skeletal dimensions.
- Source scopes are curated declarations; this offline audit does not verify publisher text.
