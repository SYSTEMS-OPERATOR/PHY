# Wooden Armature Bible PDF Versions

This directory stores preserved PDF render artifacts for the Wooden Armature Bible.

The working output remains:

```text
PROJECTS/reports/WOODEN_ARMATURE_BIBLE.pdf
```

Versioned artifacts live here so earlier render passes are not lost when the working PDF is rebuilt.

## Versioning doctrine

Each PDF version should preserve:

- the PDF file
- the PDF build report if available
- a short metadata JSON record
- a manifest entry

## Naming pattern

```text
WOODEN_ARMATURE_BIBLE_v001_original_relic_draft.pdf
WOODEN_ARMATURE_BIBLE_v001_original_relic_draft.pdf_report.json
WOODEN_ARMATURE_BIBLE_v001_original_relic_draft.version.json
```

## Current known artifact

The first draft render is important because it captures the original redwood/parchment relic PDF style before stateful footers and embedded SVG diagrams were added.

Recommended first version label:

```text
v001_original_relic_draft
```
