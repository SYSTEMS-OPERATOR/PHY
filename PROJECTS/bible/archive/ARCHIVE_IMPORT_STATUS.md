# Wooden Armature Bible Archive Import Status

## Purpose

This file records the current offline image-archive state for the Wooden Armature Bible.

It summarizes what has been imported into the repo and what remains source-only or pending.

## Anatomical archive

Status:

```text
COMPLETE FOR CURRENT SELECTED SET
```

Imported anatomical image count:

```text
Batch 1: 12 / 12
Batch 2: 7 / 7
Total: 19 / 19
```

Imported families:

- Vesalius
- Albinus
- Cheselden
- Eustachi
- Bourdon

Archive path:

```text
PROJECTS/bible/archive/anatomical_plates/
```

Reports:

```text
PROJECTS/bible/archive/anatomical_plates/plate_manifest_import_report.json
PROJECTS/bible/archive/anatomical_plates/plate_manifest_batch2_systems_import_report.json
```

Review file:

```text
PROJECTS/bible/archive/anatomical_plates/PLATE_REVIEW.md
```

## Charged-object archive

Status:

```text
PARTIAL AND DISCIPLINED
```

The charged-object workflow imported only entries marked:

```text
KEEP_READY_FOR_IMPORT
```

It intentionally skipped:

```text
KEEP_SOURCE_ONLY
SEEK_BETTER_SOURCE
KEEP_COMPARATIVE
```

because those entries require rights, source, or ethical hardening before binary archive import.

## Charged-object import summary

Workflow summary:

```text
Manifest count: 2
Allowed statuses: KEEP_READY_FOR_IMPORT
Error count: 0
```

Imported charged-object images:

```text
PROJECTS/bible/archive/charged_objects/nkisi/brooklyn_56_6_98_nkisi_nkondi_front.jpg
PROJECTS/bible/archive/charged_objects/thought_forms/thought_forms_color_key.jpg
PROJECTS/bible/archive/charged_objects/thought_forms/thought_forms_fig16_self_renunciation.jpg
PROJECTS/bible/archive/charged_objects/thought_forms/thought_forms_plate_g_music_gounod.jpg
```

Skipped intentionally:

- Met Nkisi Nkondi — source-only
- British Museum A'a from Rurutu — source-only
- Iyoba pendant mask — comparative, not included in first import
- poppet / effigy source — seek better source
- Japanese ningyo memorial source — seek better source
- Saint Eustace reliquary — source-only
- Limoges châsse / Becket Casket — source-only
- witch bottle / witch ladder — source-only
- bocio / bochio / Vodun source family — source-only
- Okiku haunted doll — seek better source / caution only
- kitchen witch / protective poppet — source-only

Reports:

```text
PROJECTS/bible/archive/charged_objects/charged_object_visuals_import_summary.json
PROJECTS/bible/archive/charged_objects/visual_source_manifest_import_report.json
PROJECTS/bible/archive/charged_objects/visual_source_manifest_batch2_import_report.json
```

Review file:

```text
PROJECTS/bible/archive/charged_objects/CHARGED_OBJECT_VISUAL_REVIEW.md
```

## Current archive doctrine

```text
archive only what is ready
source-link what is valuable but not rights-hardened
reject what is sensational or weak
```

The archive is now ready for the first Markdown compilation pass.

Image embedding into the PDF should wait until a future layout pass.
