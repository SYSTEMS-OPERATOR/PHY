# Wooden Armature Bible Figures

This directory contains deterministic SVG figures used by the Wooden Armature Bible.

The first figure set is intentionally plain and build-oriented:

- simple linework
- large labels
- redwood / brass / ink palette
- no hallucinated machine detail
- suitable for offline viewing
- suitable for later PDF embedding

## Figure policy

Every figure should teach a design consequence.

```text
figure -> mechanical/anatomical lesson -> fabrication consequence
```

## Current figure set

```text
axial_core_front.svg
pelvic_bowl_front.svg
shoulder_girdle_front.svg
upper_limb_chain.svg
hand_structure.svg
lower_limb_chain.svg
foot_structure.svg
joint_mechanics_sheet.svg
```

## PDF note

The first PDF builder may reference figure paths as text only. Future PDF passes should embed the SVGs after layout testing.
