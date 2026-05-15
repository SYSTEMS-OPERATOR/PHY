# WOODEN ARMATURE BIBLE PLAN

Long-form specification plan for a complete PHY wooden armature reference book.

Target output:

- human-readable technical specification
- approximately 120 to 150+ pages when fully expanded
- dense but navigable
- grounded in project-local materials, component guides, joint diagrams, validation records, and fabrication tests
- suitable for offline design review and build planning

Working title:

`THE WOODEN ARMATURE BIBLE`

## Scope

The Bible describes a buildable wooden humanoid armature as a fabrication system, not merely an anatomical skeleton.

It must cover:

- primary component assemblies
- component sub-elements
- material selection
- wood aging after refinement
- joint degrees of freedom
- joint thresholds and stops
- drilling and bushing logic
- hardware interfaces
- soft-tension routing
- sensor and signal routing
- finishing and refined aging
- validation coupons
- prototype-to-final workflow
- material-spec appendix

## Existing source files

Primary existing references:

- `PROJECTS/WOOD_BIBLE.md`
- `PROJECTS/WOOD_REFINED_AGING.md`
- `PROJECTS/reports/WOOD_COMPARISON_MATRIX.md`
- `PROJECTS/material_record.template.json`
- `PROJECTS/component_guide.template.json`
- `PROJECTS/REDWOOD/PROJECT.md`
- `PROJECTS/*/materials/*.json`

## Proposed book structure

### Front matter: 6 to 10 pages

- title page
- intent and scope
- safety and validation philosophy
- units and notation
- how to read diagrams
- glossary of fabrication terms

### Part I: System overview: 12 to 18 pages

Purpose:

Explain the armature as a layered physical system.

Chapters:

1. Armature as skeleton, mechanism, and artifact
2. Universal PHY vs project-local REDWOOD truth
3. Component hierarchy
4. Material philosophy: age, endurance, and fabrication truth
5. Prototype-to-final workflow

Required diagrams:

- full-body component hierarchy
- primary assembly map
- fabrication data flow
- material decision flow

### Part II: Primary component assemblies: 45 to 65 pages

Each primary component category receives a full chapter.

Required chapter template:

- role in the armature
- anatomical reference boundaries
- fabrication components
- suggested wood form factors
- grain direction principles
- hardware interfaces
- soft-tension interfaces
- sensor / signal routing interfaces
- degrees of freedom
- intended joint thresholds
- failure modes
- inspection checklist
- open unknowns

Primary categories:

1. Axial Core
2. Pelvic Bowl
3. Shoulder Girdle
4. Upper Limbs
5. Hands
6. Lower Limbs
7. Feet
8. Joint Hardware System
9. Soft Interface / Tension System
10. Sensor / Signal Routing System
11. Surface / Finish System
12. Validation / Test Coupons

### Part III: Joint geometry and degrees of freedom: 20 to 30 pages

Purpose:

Provide elementary geometry diagrams and motion limits for all fundamental joint types.

Required joint types:

- hinge joint
- pin joint
- ball-and-socket joint
- saddle-like joint
- sliding slot
- cam collar friction joint
- pawl lock
- limited universal joint
- torsion-limited rod joint
- elastic return joint

Required diagrams:

- one-axis rotation
- two-axis rotation
- three-axis rotation
- flexion / extension
- abduction / adduction
- internal / external rotation
- translation slot
- hard-stop arc
- soft-stop arc
- neutral pose and threshold markers

Example elementary diagram style:

```text
HINGE DOF

        rotation axis
             |
             v
    [bone A]---O---[bone B]
               )
              )  allowed arc
             )

DOF: 1 rotational
limits: min_angle_deg / max_angle_deg
stop type: hard stop or soft stop
```

### Part IV: Materials and refined aging: 20 to 30 pages

Purpose:

Explain material selection for the armature after wood has been cut, carved, drilled, sanded, sealed, and placed into service.

Chapters:

1. Wood as refined armature matter
2. Age vs endurance hypothesis
3. Moisture exchange and dimensional movement
4. End-grain sealing and surface finishing
5. Hardware staining and extractive bleed
6. Species notes: redwood, cypress, juniper, bristlecone, default woods
7. Sample-test gate before final cuts

Required diagrams:

- radial vs tangential shrinkage
- end-grain moisture path
- hardware staining interface
- bushing stress concentration
- humidity-cycle test coupon

### Part V: Build workflow and validation: 15 to 25 pages

Purpose:

Convert design theory into build-stage requirements.

Chapters:

1. Rough blank workflow
2. Finished envelope workflow
3. Drill schedule workflow
4. Hardware cut-list workflow
5. Coupon testing workflow
6. Component validation status
7. Failure conditions
8. Offline review packet

Required tables:

- build stage checklist
- validation status meanings
- required fields per component
- required fields per material
- required output artifacts

### Appendix A: Material specification appendix: 25 to 40 pages

Purpose:

Provide complete overview of all wood types and building materials in `PROJECTS/`.

Each material entry should include:

- material ID
- project
- common name
- Latin name when applicable
- grade
- source status
- density
- specific gravity
- Janka hardness
- MOR
- MOE
- crushing strength
- shrinkage values
- qualitative decay/workability/grain behavior
- availability and legal notes
- use recommendations
- unknowns
- references

Current material families:

- REDWOOD: coast redwood average, coast redwood old growth
- CYPRESS: bald cypress
- JUNIPER: western juniper
- BRISTLECONE: Great Basin bristlecone pine reference
- DEFAULT: Douglas fir, hard maple, yellow poplar

### Appendix B: Component templates and records: 10 to 20 pages

Purpose:

Document the machine-readable component schema in human terms.

Include:

- `component_guide.template.json`
- example completed component guide
- field-by-field explanation
- validation meanings
- missing-data handling

### Appendix C: Diagram library: 10 to 20 pages

Purpose:

Collect reusable elementary geometry diagrams.

Include:

- hinge DOF
- ball joint DOF
- socket cup geometry
- bushing cross section
- washer/collar stack
- cam collar
- pawl lock
- tendon anchor
- sensor pocket
- end-grain sealing
- grain-direction arrows

### Appendix D: Open questions and research backlog: 5 to 15 pages

Purpose:

Keep unresolved work explicit.

Initial backlog:

- exact fastener behavior per species
- exact finish compatibility per species
- bushing retention test results
- redwood old-growth shrinkage values
- juniper radial/tangential shrinkage split
- bristlecone engineering material table search
- legal provenance model for reclaimed/ancient stock
- full component guide population

## Document generation strategy

The final Bible should be generated from project-local files where possible.

Suggested future generated output:

- `PROJECTS/reports/WOODEN_ARMATURE_BIBLE.md`
- `PROJECTS/reports/WOODEN_ARMATURE_BIBLE.pdf`

Suggested source structure:

- `PROJECTS/bible/00_front_matter.md`
- `PROJECTS/bible/01_system_overview.md`
- `PROJECTS/bible/02_primary_components.md`
- `PROJECTS/bible/03_joint_geometry.md`
- `PROJECTS/bible/04_materials_and_aging.md`
- `PROJECTS/bible/05_build_workflow.md`
- `PROJECTS/bible/appendix_material_specs.md`
- `PROJECTS/bible/appendix_component_templates.md`
- `PROJECTS/bible/appendix_diagrams.md`
- `PROJECTS/bible/appendix_research_backlog.md`

## Completion definition

The Wooden Armature Bible is draft-complete when it contains:

- every primary component category expanded
- all current project materials summarized in the appendix
- elementary geometry diagrams for all fundamental DOF types
- intended joint threshold placeholders or values
- sample-test gate
- refined-aging model
- component record schema explanation
- known unknowns
- references

The Bible is fabrication-complete only when populated component guides, drill schedules, cut lists, and validation results exist for the chosen build material.