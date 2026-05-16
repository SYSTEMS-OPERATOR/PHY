# Wooden Armature Bible — PDF Style Guide

## Purpose

This guide defines the first visual language for `WOODEN_ARMATURE_BIBLE.pdf`.

The target is not a modern white-paper layout.

The target is a technical relic:

```text
redwood manual
anatomical bible
workshop grimoire
field book
artifact dossier
```

The PDF should feel old enough to carry ceremony, but clean enough to remain usable as an engineering reference.

## Motif

Primary motif:

```text
redwood + parchment + brass + ink
```

Design associations:

- redwood heartwood
- warm bark shadow
- aged parchment
- oxidized brass
- black-brown ink
- anatomical plate captions
- scripture-like serif gravity without blackletter clutter

## Palette

Approximate palette:

| Role | Name | Hex |
|---|---|---|
| page background | aged parchment | `#F4EAD2` |
| main ink | iron gall ink | `#241811` |
| heading | redwood heart | `#6E2F1A` |
| accent | brass line | `#B08A3C` |
| soft accent | sapwood tan | `#C9A66B` |
| code/table backing | pale vellum | `#FFF8E7` |
| shadow | bark umber | `#3B2115` |

## Fonts

Do not commit font binaries.

Use standard PDF-safe fonts for the first pass:

```text
Times-Roman
Times-Bold
Times-Italic
Courier
Helvetica
```

The intended feel is biblical / old-testament / field-manual serif, not decorative blackletter.

Future typography pass may use open-license fonts only if license and embedding rules are clean.

## Page style

Recommended first-pass page style:

- US Letter page size
- parchment background
- thin redwood outer border
- brass inner rule
- running header with book title
- footer with `MIND | BODY | SOUL` and page number
- section breaks separated by ornamental rules
- code blocks on pale vellum backing
- tables styled with redwood header background and brass grid

## Title page

The title page should include:

```text
THE WOODEN ARMATURE BIBLE
A PHY-compatible fabrication reference for wooden humanoid armature design
MIND | BODY | SOUL
```

Suggested title-page elements:

- redwood-grain abstract linework
- brass rectangular frame
- small seal text: `BONES FIRST / TRUTH FIRST / RECORDS FIRST`
- generated timestamp in small type

## Image policy

First PDF pass may be text-only with reviewed image paths and captions.

Image-rich PDF must wait for:

- rights review
- figure placement plan
- caption style review
- image resolution audit
- cultural-context review for charged-object plates

## Current PDF target

First PDF target:

```text
PROJECTS/reports/WOODEN_ARMATURE_BIBLE.pdf
```

Build script:

```text
PROJECTS/bible/build_pdf.py
```

Workflow:

```text
.github/workflows/build-wooden-armature-bible-pdf.yml
```
