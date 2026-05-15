# Appendix D — Research Backlog and Open Questions

## D.1 Purpose

This appendix keeps unresolved work visible.

Unknowns are not defects if they are tracked. They become defects only when treated as known.

## D.2 Material-property backlog

### REDWOOD

Open questions:

- exact fastener behavior for planned pin, bushing, and washer interfaces
- finish compatibility by finish type
- extractive bleed under chosen finish
- nonferrous and stainless hardware staining behavior
- old-growth-specific shrinkage values
- reclaimed-stock provenance model

Needed tests:

- redwood drill tearout coupon
- redwood bushing retention coupon
- redwood finish adhesion coupon
- redwood hardware stain coupon
- redwood humidity-cycle coupon

### CYPRESS

Open questions:

- sinker cypress vs standard bald cypress mechanical differences
- pecky cypress suitability or rejection criteria
- finish behavior on greasy-feeling raw surfaces
- bushing retention in damp-risk conditions

Needed tests:

- surface prep and finish coupon
- damp exposure coupon
- bushing retention coupon
- cutter tearout coupon

### JUNIPER

Open questions:

- radial shrinkage value
- tangential shrinkage value
- knot rejection criteria
- checking behavior around drilled holes
- suitability for small members and inserts

Needed tests:

- predrill coupon
- knot and grain rejection sample set
- humidity movement coupon
- bushing retention coupon
- small-part splitting coupon

### BRISTLECONE

Open questions:

- exact Janka hardness
- exact MOR
- exact MOE
- exact crushing strength
- exact radial shrinkage
- exact tangential shrinkage
- exact volumetric shrinkage
- whether any complete engineering material table exists
- legal provenance model for any non-protected sample material

Current status:

- reference-only
- not fabrication-authorized
- useful for endurance hypothesis and age-growth literature

### DEFAULT

Open questions:

- which DEFAULT material is best for first full-body mockup
- whether plywood should be added as a sheet/plate baseline
- whether MDF should be formally restricted to templates only
- whether hard maple is useful for inserts or too split-prone at small scale

Needed tests:

- Douglas fir hinge prototype
- hard maple insert prototype
- yellow poplar shape-study prototype
- plywood plate prototype if sheet goods are added

## D.3 Joint geometry backlog

Needed diagrams and records:

- shoulder suspension joint
- hip ball-and-socket threshold map
- knee hinge stack
- elbow hinge stack
- wrist compound joint
- ankle limited universal joint
- finger hinge cascade
- thumb saddle-like joint
- pelvis socket block
- scapula/clavicle suspension map
- bushing cross section with real dimensions
- washer/collar compression map
- cam collar friction adjuster
- pawl lock geometry

Needed threshold data:

- neutral angle
- minimum angle
- maximum angle
- soft-stop onset
- hard-stop point
- friction setting range
- service access requirements

## D.4 Component-guide backlog

First REDWOOD component records to create:

- femur test component
- humerus test component
- rib template
- phalanx template
- scapula plate
- pelvis socket block
- hinge coupon
- bushing retention coupon
- finish and hardware stain coupon

Each guide must include:

- component ID
- source bone ID where applicable
- material ID
- finished envelope
- rough blank
- grain direction
- joint centers
- hardware assignments
- drill schedule references
- adjacent connections
- simulation values when available
- fabrication allowance
- validation status
- unknowns
- references

## D.5 Cut-list and drill-schedule backlog

Needed files:

- `PROJECTS/REDWOOD/cutlists/redwood_cut_list.csv`
- `PROJECTS/REDWOOD/cutlists/drill_schedule.csv`
- `PROJECTS/REDWOOD/cutlists/hardware_cut_list.csv`

Needed CSV columns for cut list:

- component_id
- material_id
- quantity
- rough_length_mm
- rough_width_mm
- rough_thickness_mm
- finished_length_mm
- finished_width_mm
- finished_thickness_mm
- grain_direction
- fabrication_allowance_mm
- notes

Needed CSV columns for drill schedule:

- component_id
- hole_id
- diameter_mm
- depth_mm
- through_hole
- axis
- reference_origin
- offset_x_mm
- offset_y_mm
- offset_z_mm
- hardware_id
- fit_type
- notes

Needed CSV columns for hardware cut list:

- hardware_id
- type
- material
- quantity
- diameter_mm
- length_mm
- thickness_mm
- associated_component_id
- finish_or_coating
- notes

## D.6 Validation backlog

Needed validation categories:

- material record completeness
- component guide completeness
- rough blank dimensions present
- finished envelope dimensions present
- grain direction present
- joint centers present
- hardware assignments present
- drill schedule refs present
- coupon tests linked
- unknowns listed
- final-cut authorization status

## D.7 Bible generation backlog

Needed generator behavior:

- collect Bible source chapters in order
- inject material matrix from JSON records
- include diagrams in monospaced blocks
- preserve unknowns as unknown
- emit Markdown report
- optionally emit PDF artifact

Suggested future output paths:

- `PROJECTS/reports/WOODEN_ARMATURE_BIBLE.md`
- `PROJECTS/reports/WOODEN_ARMATURE_BIBLE.pdf`

## D.8 Current build recommendation

Do not begin final REDWOOD cuts until the following exist:

- REDWOOD material coupon results
- first component guides
- drill schedule format
- hardware stack format
- finish/stain coupon results
- bushing retention coupon results

Safe next physical work:

- coupon tests
- DEFAULT prototypes
- REDWOOD sample blocks
- non-final joint studies
