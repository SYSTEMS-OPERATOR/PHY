# Wooden Armature Bible Source

This directory contains modular source chapters for the long-form `WOODEN_ARMATURE_BIBLE` build artifact.

The target final artifact is a dense human-readable technical specification of approximately 120 to 150+ pages when fully expanded.

## Current source order

Canonical compile order is defined in:

```text
PROJECTS/bible/build_order.json
```

Current first-pass build order:

1. `00_front_matter.md`
2. `01_system_overview.md`
3. `02_primary_components.md`
4. `03_joint_geometry.md`
5. `04_materials_and_aging.md`
6. `05_build_workflow.md`
7. `appendix_material_specs.md`
8. `appendix_ancient_wood_carpentry.md`
9. `appendix_wooden_skeleton_precedents.md`
10. `appendix_wooden_companion_precedents.md`
11. `appendix_wooden_companion_chronological_deep_dive.md`
12. `appendix_wooden_tulpa_precedents.md`
13. `appendix_anatomical_reference_plates.md`
14. `archive/anatomical_plates/PLATE_REVIEW.md`
15. `appendix_companion_visual_reference_index.md`
16. `appendix_wooden_tulpa_visual_reference_index.md`
17. `archive/charged_objects/CHARGED_OBJECT_VISUAL_REVIEW.md`
18. `archive/ARCHIVE_IMPORT_STATUS.md`
19. `appendix_diagrams.md`
20. `appendix_research_backlog.md`
21. `BIBLE_SHAPE_REVIEW.md`
22. `appendix_x_mind_body_soul_decoder.md`

## Source policy

- Keep universal PHY logic species-agnostic.
- Keep REDWOOD-specific assumptions local to `PROJECTS/REDWOOD/` or explicitly marked as project-local.
- Do not invent missing measurements.
- Use `null`, TBD, or explicit unknown language where data is incomplete.
- Prefer short reusable diagrams that survive plain-text offline use.
- Every fabrication claim should point back to a material record, component guide, test coupon, validation output, or explicit source-review note.
- Image files may be archived locally only when their manifest review status permits import.
- Source-only visual references must remain source-only until rights, quality, and context are hardened.

## Generation targets

Generated outputs:

- `PROJECTS/reports/WOODEN_ARMATURE_BIBLE.md`
- `PROJECTS/reports/WOODEN_ARMATURE_BIBLE.build_report.json`

Future generated outputs:

- `PROJECTS/reports/WOODEN_ARMATURE_BIBLE.pdf`

## Current status

The first compiled Markdown artifact exists and builds with zero missing sources.

The current artifact is a compiled draft, not fabrication-complete and not final-layout-complete.

Image embedding and PDF layout are future passes after plate rights and layout review.
