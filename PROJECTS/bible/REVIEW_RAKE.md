# Wooden Armature Bible — Review Rake

## Purpose

This file records the first review-rake pass after the initial compiled Markdown artifact was generated.

The review-rake checks:

- build report health
- source order health
- README/build-order drift
- archive readiness
- obvious compile hygiene
- remaining weak points before PDF/layout work

## Build health

Current compiled artifact:

```text
PROJECTS/reports/WOODEN_ARMATURE_BIBLE.md
```

Build report:

```text
PROJECTS/reports/WOODEN_ARMATURE_BIBLE.build_report.json
```

Observed build result:

```text
section_count: 21
missing_sources: []
```

Rake verdict:

```text
PASS
```

The first compiled artifact has no missing sources.

## Rake finding 1 — README source order was stale

Finding:

The Bible README still listed the early short scaffold source order and did not reflect the expanded 21+ section build order.

Action:

Updated `PROJECTS/bible/README.md` to point to `build_order.json` as canonical and list the current review-rake source order.

Status:

```text
FIXED
```

## Rake finding 2 — Archive status should be included in compile order

Finding:

The archive image imports had succeeded and were documented in `archive/ARCHIVE_IMPORT_STATUS.md`, but that status file was not included in the compiled Bible.

Action:

Updated `PROJECTS/bible/build_order.json` to include:

```text
archive/ARCHIVE_IMPORT_STATUS.md
```

immediately after `archive/charged_objects/CHARGED_OBJECT_VISUAL_REVIEW.md` and before the diagrams/research backlog.

Status:

```text
FIXED
```

## Rake finding 3 — Build order status needed update

Finding:

`build_order.json` still called itself `first compile order` even after review-rake changes.

Action:

Changed status to:

```text
review-rake compile order
```

and clarified that image embedding remains deferred until rights/layout review.

Status:

```text
FIXED
```

## Rake finding 4 — Plate archive discipline is working

Finding:

Anatomical plate archive is complete for the current selected set.

Charged-object archive is partial but disciplined: it imported only `KEEP_READY_FOR_IMPORT` entries and skipped source-only / seek-better-source / comparative entries.

Action:

No correction needed.

Status:

```text
PASS
```

## Rake finding 5 — Core technical chapters are coherent but still scaffold-level

Finding:

The technical core is coherent:

- system overview
- primary components
- joint geometry
- material aging
- build workflow

But the core remains a framework, not a final fabrication package.

Still needed after compiled-book review:

- complete REDWOOD-specific component examples
- drill schedule CSV draft
- hardware stack examples
- coupon result templates
- actual test result records
- generated diagrams / SVGs

Status:

```text
KNOWN GAP
```

## Rake finding 6 — Image embedding should remain deferred

Finding:

Image archives exist, but the compiled Bible does not embed images yet.

This is correct. Rights review, layout selection, caption style, and PDF design should happen before embedding.

Status:

```text
PASS
```

## Rake finding 7 — Appendix X is correctly placed as final seal

Finding:

Appendix X belongs at the end. It functions as the symbolic decoder after the technical, historical, visual, and archive sections.

Status:

```text
PASS
```

## Current corrected build order

The corrected build order now contains 22 sections:

```text
00_front_matter.md
01_system_overview.md
02_primary_components.md
03_joint_geometry.md
04_materials_and_aging.md
05_build_workflow.md
appendix_material_specs.md
appendix_ancient_wood_carpentry.md
appendix_wooden_skeleton_precedents.md
appendix_wooden_companion_precedents.md
appendix_wooden_companion_chronological_deep_dive.md
appendix_wooden_tulpa_precedents.md
appendix_anatomical_reference_plates.md
archive/anatomical_plates/PLATE_REVIEW.md
appendix_companion_visual_reference_index.md
appendix_wooden_tulpa_visual_reference_index.md
archive/charged_objects/CHARGED_OBJECT_VISUAL_REVIEW.md
archive/ARCHIVE_IMPORT_STATUS.md
appendix_diagrams.md
appendix_research_backlog.md
BIBLE_SHAPE_REVIEW.md
appendix_x_mind_body_soul_decoder.md
```

## Recommended next action

Run the build workflow again.

Expected new build report:

```text
section_count: 22
missing_sources: []
```

After that:

1. inspect the compiled Markdown around section boundaries
2. decide whether to make a first non-image PDF
3. defer image-rich PDF until layout and rights pass

## Final rake verdict

The book is structurally sound.

The archive is disciplined.

The next compile should be clean.

The main remaining work is no longer source recovery. It is refinement:

```text
bridge text
component examples
diagrams
image layout
PDF typography
fabrication tables
```

The skeleton stands. The next pass teaches it to read as a book.
