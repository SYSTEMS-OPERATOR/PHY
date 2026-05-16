# Part II Expansion Plan — Primary Component Assemblies

## Purpose

Part II is the mechanical and anatomical heart of the Wooden Armature Bible.

The review-rake found that the current Part II is coherent, but too compressed. It names the primary assemblies without giving each assembly enough space to become a design guide.

This plan defines the expansion target.

## Desired outcome

Each primary component section should become equal parts:

```text
visual representation
anatomical explanation
mechanical function
fabrication strategy
validation checklist
system-state contribution
```

A reader should be able to browse the figures and understand the design intention before reading the detailed prose.

## Section structure template

Each primary component section should include:

1. **Role and doctrine**
2. **Anatomical boundaries**
3. **Average human proportion cues**
4. **Buildable subcomponents**
5. **Form logic**
6. **Function logic**
7. **Evolutionary / biomechanical pressure**
8. **Material and grain strategy**
9. **Joint interfaces encountered**
10. **Soft-tension / ligament analogues**
11. **Sensor / service routing**
12. **Fabrication procedure**
13. **Visual reference figure**
14. **Failure modes**
15. **Validation tests**
16. **System-state contribution**

## Component sections requiring expansion

```text
2.2 Axial Core
2.3 Pelvic Bowl
2.4 Shoulder Girdle
2.5 Upper Limbs
2.6 Hands
2.7 Lower Limbs
2.8 Feet
2.9 Joint Hardware System
2.10 Soft Interface / Tension System
2.11 Sensor / Signal Routing System
2.12 Surface / Finish System
2.13 Validation / Test Coupons
```

## Illustration policy

Use clean deterministic diagrams first:

- black/brown linework
- redwood/bone silhouette accents
- brass joint circles
- labels large enough for print
- no AI-render hallucinated mechanics
- no decorative mechanism that cannot be explained

Use sourced historical or educational mechanical figures only when they are rights-clear, relevant, and superior to a custom diagram.

## Mechanical figure types needed

```text
revolute / pin joint
ball-and-socket envelope
limited universal joint
washer/bushing stack
cam collar friction clamp
pawl lock / ratchet arc
sliding slot
elastic return line
soft stop vs hard stop arc
exploded assembly view
free-body / load path diagram
kinematic diagram
```

## External reference anchors

The visual language should be informed by established engineering diagram traditions:

- kinematic diagrams: show links and joints/connectivity rather than final part shape
- exploded views: show assembly order and part relationships
- free-body diagrams: show force and moment relationships
- simple machines / kinematic chains: useful for reducing joints to elementary motion logic

These are reference concepts, not necessarily imported image files.

## PDF requirement

Major new chapters and major component sections should begin on their own page in the PDF.

Footer tags should reflect the active section state:

```text
BODY = material, anatomy, components, joints, build workflow
MIND = archives, manifests, plate reviews, build reports, source logic
SOUL = companion history, charged-object history, tulpa-adjacent material
MIND | BODY | SOUL = Appendix X and synthesis sections
```

## First expansion pass

First pass should:

1. add deterministic SVG diagrams under `PROJECTS/bible/figures/`
2. add a figure index
3. expand Part II sections using figure references
4. update the PDF builder to assign relevant footer states by section title
5. force top-level sections onto new pages
6. later: add image embedding to PDF after SVG/image rendering pipeline is stable
