# Wooden Armature Bible Source

This directory contains modular source chapters for the future long-form `WOODEN_ARMATURE_BIBLE` build artifact.

The target final artifact is a dense human-readable technical specification of approximately 120 to 150+ pages when fully expanded.

## Source order

1. `00_front_matter.md`
2. `01_system_overview.md`
3. `02_primary_components.md`
4. `03_joint_geometry.md`
5. `04_materials_and_aging.md`
6. `05_build_workflow.md`
7. `appendix_material_specs.md`
8. `appendix_diagrams.md`
9. `appendix_research_backlog.md`

## Source policy

- Keep universal PHY logic species-agnostic.
- Keep REDWOOD-specific assumptions local to `PROJECTS/REDWOOD/` or explicitly marked as project-local.
- Do not invent missing measurements.
- Use `null`, TBD, or explicit unknown language where data is incomplete.
- Prefer short reusable diagrams that survive plain-text offline use.
- Every fabrication claim should point back to a material record, component guide, test coupon, or validation output.

## Generation targets

Future generated outputs:

- `PROJECTS/reports/WOODEN_ARMATURE_BIBLE.md`
- `PROJECTS/reports/WOODEN_ARMATURE_BIBLE.pdf`

## Current status

This is the drafting scaffold. It is not yet fabrication-complete.
