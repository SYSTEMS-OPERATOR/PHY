# Part I — System Overview

## 1.1 Armature as layered physical system

The wooden armature is a layered fabrication system:

```text
ARMATURE
  anatomy layer
  material layer
  joint hardware layer
  soft-tension layer
  sensor-routing layer
  surface/finish layer
  validation layer
```

The anatomy layer defines shape and proportion.

The material layer defines what each part is made from and how that material behaves after refinement.

The joint hardware layer defines pins, bushings, collars, washers, hinge plates, stops, spacers, and adjustable friction elements.

The soft-tension layer defines elastic restraint, return behavior, anti-flop support, and pose feel.

The sensor-routing layer defines future channels for wiring, strain sensing, pressure sensing, connector access, and repair.

The surface/finish layer defines touch quality, end-grain sealing, UV exposure behavior, staining risk, and long-term maintenance.

The validation layer proves whether the component can be trusted.

## 1.2 Universal PHY vs project-local truth

Universal PHY remains species-agnostic.

Project overlays provide local build truth.

```text
PHY CORE
  skeleton schema
  canonical bone records
  validation logic
  export logic

PROJECTS
  wood bible
  material records
  refined-aging rules
  project-specific components
  cut lists
  drill schedules
  hardware schedules
  reports
```

REDWOOD is the primary build candidate, but the specification must remain comparative. CYPRESS, JUNIPER, BRISTLECONE, and DEFAULT exist to test assumptions and prevent premature material worship.

## 1.3 Primary assembly hierarchy

The armature is divided into fabrication-primary assemblies:

```text
ARMATURE
  AXIAL_CORE
  PELVIC_BOWL
  SHOULDER_GIRDLE
  UPPER_LIMBS
  HANDS
  LOWER_LIMBS
  FEET
  JOINT_HARDWARE
  SOFT_TENSION_SYSTEM
  SENSOR_SIGNAL_ROUTING
  SURFACE_FINISH_SYSTEM
  VALIDATION_COUPONS
```

These categories are not merely anatomical. They describe how the object is built, tested, assembled, serviced, and aged.

## 1.4 Component as fabrication unit

A component is any buildable physical item with a record.

Examples:

- femur blank
- scapula plate
- pelvis bridge
- rib segment
- finger phalanx
- brass bushing
- copper collar
- washer stack
- tendon anchor
- sensor pocket
- finish coupon
- drilled test block

Every component should eventually map to a `component_guide` record.

## 1.5 Material decision flow

Material selection is not a single ranking. It is a role-specific decision.

```text
role requirement
      |
      v
material record available?
      |
      +-- no --> use DEFAULT prototype or mark unknown
      |
      v
enough mechanical values?
      |
      +-- no --> reference-only or test-only
      |
      v
refined-aging risks acceptable?
      |
      +-- no --> test coupon / alternate material
      |
      v
legal and repeatable source?
      |
      +-- no --> not fabrication-ready
      |
      v
component guide allowed
```

## 1.6 Prototype-to-final workflow

The preferred build path is staged:

1. DEFAULT mockup
2. material coupon tests
3. REDWOOD draft component
4. hardware interface test
5. refined-aging check
6. final component cut
7. validation report
8. build book inclusion

DEFAULT materials are not failures. They are sacrificial clarity.

## 1.7 Current material posture

Current strongest visible build candidate:

- coast redwood

Current endurance challengers:

- bald cypress
- western juniper

Current prototype baselines:

- Douglas fir
- hard maple
- yellow poplar

Current age-reference control:

- Great Basin bristlecone pine

Bristlecone is not currently a cut-stock material in this project. It is an endurance-design reference.

## 1.8 Design consequence

The Bible must avoid a false equation:

```text
old tree = better part
```

The correct equation is:

```text
old tree = evidence of survival traits
survival traits + verified properties + refined-aging tests = possible design value
```

The armature should be built from what can be verified, tested, sourced, and maintained.
