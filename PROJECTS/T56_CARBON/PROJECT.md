# T-5.6 CARBON Project Overlay

T-5.6 CARBON is a PHY-compatible project overlay for a 5 ft 6 in (1676.4 mm) domestic humanoid armature built around hybrid carbon-composite members, serviceable metallic joint cartridges, tendon-driven distal mechanisms, and compliant artificial fascia.

This project does not redefine universal PHY anatomy or fabrication policy. It supplies one concrete embodiment target using project-local proportions, materials, interfaces, validation plans, component architecture, and simulation artifacts.

## Identity

- Project ID: `T56_CARBON`
- Height: `1676.4 mm`
- Baseline arm span: `1676.4 mm`
- Lineage: mature domestic evolution of the utilitarian `T-5.5(5)` concept
- Pose baseline: neutral anatomical T-pose
- Frame class: hybrid carbon-composite armature
- Intended environment: quiet indoor and domestic operation

## Design intent

T-5.6 prioritizes:

- quiet movement and low vibration
- safe close-proximity operation
- comfortable sitting, reclining, and bedside posture
- serviceable joints and replaceable wear components
- high tactile-sensor density
- guarded pinch points and compliant contact surfaces
- adult, capable proportions without exaggerated display anatomy

## Structural doctrine

> Carbon carries the frame. Metal carries the joints. Tension carries the body. Softness carries the contact.

- Carbon-composite members carry distributed bending and torsional loads.
- Metallic cartridges carry bearings, threads, shafts, actuator mounts, and concentrated loads.
- Bonded inserts use keyed geometry, generous bond length, load spreading, and secondary mechanical capture.
- No raw bolt loads directly through unreinforced laminate.
- No direct carbon-to-aluminum contact without a dielectric isolation layer.
- Artificial fascia distributes tension, damps vibration, supports outer-body volume, and provides sensor attachment structure.

## Architecture

### Long members

Femur, tibia, fibula, humerus, radius, and ulna use open or closed carbon-composite truss members. Initial prototypes may use commercial structural tubing where it reduces manufacturing uncertainty.

### Torso

The torso is layered:

1. central spine and pelvic load path
2. shoulder and rib structural frame
3. actuator and electronics cradle
4. removable carbon service panels
5. compliant fascia and outer body

### Pelvis

The pelvis is not monolithic. It consists of:

- central sacral load bridge
- left and right iliac composite shells
- separate replaceable hip cartridges
- compliant tensile stabilization network

### Hands and feet

Hands use light distal linkages with tendons routed to serviceable forearm actuators. Feet use a broader hidden sole structure, pressure sensing, compliant heel/toe elements, and replaceable contact surfaces.

## MVP demonstrator

The first complete systems article is the left forearm:

- `BONE_RADIUS_L` carbon member
- `BONE_ULNA_L` carbon member
- elbow and wrist cartridges
- internal service and cable channels
- tactile electrode segments
- skin/fascia mount points
- destructive coupons made from the same material, adhesive, and insert geometry

## Fabrication status

Status: `SCAFFOLD_NOT_FABRICATION_READY`

All laminate schedules, dimensions, material constants, adhesives, insert geometries, load ratings, joint limits, and safety factors remain unvalidated until supported by traceable references and physical coupon testing.

Unknown values must be marked explicitly. No fabrication-critical value may be inferred from visual concept art.

## Required artifacts

The project becomes MVP-ready only when it can generate or store:

- complete project profile and proportion rules
- referenced material records
- bonded-insert and joint-cartridge specifications
- forearm fabrication packet
- coupon and joint validation reports
- cutlists and drill schedules
- mass and inertia estimates
- URDF-like simulation output
- service-access and cable-routing documentation
