# T-5.6 Floating Scapular Girdle and Shoulder Cartridge Architecture

Status: `CONCEPT_SCAFFOLD_NOT_FABRICATION_READY`

This document defines the project-local shoulder architecture that branches from the merged T-5.6 nested thorax. It uses the supplied concept plates only as topology references; no pixel-derived dimensions, copied surface geometry, or assumed mechanism details are treated as engineering truth.

> Spine roots. Scapulae travel. Clavicles guide. Cartridges carry the arms. Softness guards the motion.

## Coordinate frame

The shoulder assembly inherits the thoracic coordinate frame:

- origin: `thoracic_midline_at_sternum_spine_reference_plane`
- `x`: left to right
- `y`: posterior to anterior
- `z`: inferior to superior

All shoulder coordinates, travel envelopes, joint centers, and collision volumes remain `null` until the T-5.6 thoracic anthropometric envelope is locked.

## Assembly stack

1. **Thoracic root interface** — validated hardpoints on the upper inner thoracic housing and segmented spine.
2. **Bilateral scapular carriages** — movable structural carriages that position each lateral shoulder center without asking the secondary carbon ribcage to carry primary arm reactions.
3. **Bilateral clavicular linkages** — anterior-superior control links that constrain and share carriage motion.
4. **Lateral shoulder yokes** — compact outboard structures joining each carriage and clavicular linkage to a removable shoulder cartridge.
5. **Humeral shoulder cartridges** — replaceable bearing, shaft, sensing, hard-stop, and actuator-interface modules.
6. **Sensors, guards, routing, and fascia interfaces** — serviceable systems that preserve motion while protecting cables, skin, and nearby people.

## Primary load path

The intended reaction path is:

`arm and humeral member -> humeral shoulder cartridge -> lateral shoulder yoke -> scapular carriage and clavicular linkage -> thoracic root nodes -> inner thoracic housing and segmented spine`

The secondary carbon ribcage, compliance padding, and outer form are excluded from the primary shoulder load path unless separately analyzed and physically validated.

## Kinematic intent

The assembly must provide controlled machine equivalents of:

- scapular protraction and retraction
- elevation and depression
- upward and downward rotation
- limited anterior and posterior tilt
- shoulder-center translation associated with arm elevation

The architecture does not require literal replication of biological scapular sliding. A compact guided carriage, coupled linkage, or hybrid mechanism may be used if it reproduces the required shoulder-center trajectory, load transfer, serviceability, and domestic safety behavior.

Left and right assemblies share topology but require independent calibration. Visual symmetry is not proof of equal stiffness, friction, travel, or sensor zero.

## Mechanical boundaries

- Every moving carriage requires positive retention and redundant travel limits.
- No single actuator, cable, sensor, software limit, or cosmetic panel may serve as the only motion stop.
- Carbon members may not serve directly as bearing races.
- Concentrated cartridge loads must enter validated metal nodes or reinforced composite inserts.
- Direct carbon-to-aluminum contact remains prohibited without a validated dielectric system.
- Exposed telescoping shafts, pinch gaps, and shear edges are prohibited in the domestic contact envelope.
- A shoulder cartridge must be removable without disturbing thoracic alignment or cutting fascia.

## Domestic adaptation

The concept-reference topology is translated into a quieter and narrower T-5.6 body envelope:

- compact shoulder breadth rather than an armored upper torso
- restrained clavicle rise and natural shoulder slope
- enclosed or guarded carriage motion
- low distal mass
- compliant fascia bridges over the scapular and clavicular mechanisms
- rounded service covers with no load-bearing cosmetic spikes
- safe reclining and side-contact behavior
- cable routing that tolerates repeated arm elevation without abrasion or snagging

## Sensor doctrine

At minimum, the architecture reserves channels for:

- shoulder-cartridge absolute position
- carriage position or equivalent linkage state
- root-node strain or reaction load
- cartridge temperature
- vibration and impact events
- hard-stop contact detection
- cable or tendon strain where routed through the shoulder region
- service-panel and guard presence

Sensor placement must not weaken primary members or obstruct cartridge removal.

## Service doctrine

The left and right shoulder assemblies must be independently serviceable. Required access includes:

- posterior carriage guide and root fasteners
- anterior clavicular-link roots
- lateral shoulder cartridge retention
- cable and tendon service loops
- sensor connectors and zeroing features
- guards, fascia bridges, and compliant outer supports
- inspection access to dielectric barriers and wear surfaces

No shoulder service task may require removal of the central electronics core unless the failed component is part of that core.

## Release blockers

- locked T-5.6 thoracic coordinate frame and shoulder envelope
- validated shoulder-center trajectory
- primary arm and impact load cases with safety factors
- selected root-node, carriage, linkage, bearing, and insert materials
- resolved bearing, guide, and wear geometry
- hard-stop and passive-retention design
- cable, tendon, thermal, and sensor routing
- collision analysis against neck, ribcage, outer form, and arm
- static, cyclic, impact, acoustic, and service validation
- paired-shoulder testing on a representative thoracic subassembly

## Design sentence

> Spine roots. Scapulae travel. Clavicles guide. Cartridges carry the arms. Softness guards the motion.
