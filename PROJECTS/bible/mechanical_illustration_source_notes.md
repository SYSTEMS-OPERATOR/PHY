# Mechanical Illustration Source Notes

## Purpose

This note records the visual-reference logic behind the expanded mechanical diagrams used in Part II, Part III, and Appendix C.

The project needs figures that are:

- clear
- exact enough to teach
- fabrication-relevant
- browseable without prose
- not dependent on lucky AI renders
- not copied blindly from rights-unclear sources

## Current decision

Use deterministic custom SVG diagrams as the primary figure layer.

Do not import random mechanical illustrations until rights and relevance are clear.

## Reference doctrine

### Kinematic diagrams

A kinematic diagram shows mechanism connectivity: links, joints, and motion relationships, not cosmetic part shape.

This is the correct doctrine for PHY joint drawings because the reader needs to understand:

- which parts move
- what kind of joint is present
- how many degrees of freedom exist
- where hard stops and soft stops act
- what must be fabricated or validated

Use kinematic diagram language for:

- hinge / pin joints
- sliding slots
- ball-and-socket envelopes
- limited universal joints
- tendon/soft-tension paths
- washer/bushing stack simplifications

### Four-bar and linkage traditions

Four-bar linkage diagrams are useful reference anchors because they reduce mechanisms to links and joints.

Use this tradition when future figures require:

- rocker motion
- coupled motion
- slider-crank analogy
- knee/ankle assisted linkages
- cam or return mechanisms

### Engineering drawing / exploded view tradition

Exploded views are useful when the fabrication question is assembly order rather than motion.

Use exploded-view doctrine for:

- washer/bushing stack
- pelvis hip socket insert
- shoulder cup stack
- finger micro-joint stack
- foot sole pad assembly

### Free-body / load-path tradition

Free-body diagrams are useful when the fabrication question is force path.

Use load-path figures for:

- pelvic seated load
- standing lower-limb compression
- shoulder suspended-arm load
- foot ground reaction
- tension-line anchoring

## External source anchors to harden later

These are reference directions, not imported image approvals:

- Cornell / KMODDL mechanism model tradition and Reuleaux-style kinematic model collections
- simple kinematic diagram definitions and examples
- four-bar linkage and slider-crank diagram conventions
- simple-machine educational diagrams from official or open educational sources
- engineering drawing and exploded-view guides from rights-clear educational sources

## Current custom figure set

```text
PROJECTS/bible/figures/axial_core_front.svg
PROJECTS/bible/figures/pelvic_bowl_front.svg
PROJECTS/bible/figures/shoulder_girdle_front.svg
PROJECTS/bible/figures/upper_limb_chain.svg
PROJECTS/bible/figures/hand_structure.svg
PROJECTS/bible/figures/lower_limb_chain.svg
PROJECTS/bible/figures/foot_structure.svg
PROJECTS/bible/figures/joint_mechanics_sheet.svg
```

## Future hardening

Later source hardening should seek rights-clear examples for:

- revolute/prismatic joint symbols
- exploded mechanical assembly drawings
- cross-section bushing diagrams
- free-body load-path diagrams
- simple machines / kinematic chain teaching diagrams

Do not import a figure unless it beats the custom SVG on clarity and has a clean rights path.
