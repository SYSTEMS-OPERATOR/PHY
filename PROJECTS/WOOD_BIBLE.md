# WOOD BIBLE

Shared comparison reference for PHY wood-material projects.

Projects governed by this reference:

- DEFAULT
- REDWOOD
- BRISTLECONE
- JUNIPER
- CYPRESS

## Purpose

Compare wood species and grades for PHY armature fabrication using exact sourced material properties, explicit unknowns, legal sourcing notes, and component-use scoring.

This reference is not a generic lumber guide. It is a project-level comparison frame for small, shaped, drilled, carved, pinned, and hardware-coupled armature components.

## Layer rule

The WOOD BIBLE defines shared comparison vocabulary.

Species projects define project-local data.

Universal PHY files remain species-agnostic:

- skeleton/base.py
- skeleton/schema/bone.schema.json
- skeleton/bones/
- BODY.md
- MIND.md

## Required evaluation categories

Exact or source-backed fields:

- density
- specific gravity
- hardness
- modulus of rupture
- modulus of elasticity
- crushing strength
- radial shrinkage
- tangential shrinkage
- volumetric shrinkage
- decay resistance
- workability
- fastener behavior
- grain behavior
- legal availability
- long-duration endurance traits

## Source status

Each material property must be marked as one of:

- verified: exact value from a cited source
- partial: incomplete but source-backed data exists
- estimated: reasoned project estimate, not fabrication truth
- missing: no acceptable value found yet

## Ancient-grade tolerance concept

Ancient-grade does not mean stronger by default.

Ancient-grade means the material has evidence of long-duration endurance traits such as slow growth, tight ring structure, mature heartwood, decay resistance, extractive content, dimensional stability, resin or mineral character, and survival under long-duration stress.

Ancient-grade claims must separate measured mechanical properties from interpretation.

## Component-use scoring

Scores are 0 to 100 and must distinguish evidence from judgement.

Shared categories:

- long members
- flat members
- small members
- joint blocks
- sensor channels
- anchor zones
- pin and bushing interfaces
- surface finish
- repairability
- endurance value

## Validation policy

No unknown value should be invented as fabrication truth.

A material record is incomplete until it has exact values where available, explicit null values where unknown, source status labels, legal sourcing notes, and references or an explicit missing-data note.
