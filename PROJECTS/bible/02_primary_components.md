# Part II — Primary Component Assemblies

## 2.1 Category template

Each primary component category should eventually include:

- assembly role
- anatomical boundaries
- buildable subcomponents
- suggested material role
- grain direction principle
- hardware interfaces
- soft-tension interfaces
- sensor-routing interfaces
- expected DOF
- intended thresholds
- failure modes
- validation tests
- open unknowns

## 2.2 Axial Core

Role:

The axial core is the centerline structure. It carries posture, torso height, head support, rib connection, and pelvis connection.

Buildable elements:

- skull support or head mount
- cervical stack
- thoracic stack
- lumbar stack
- sacrum interface
- sternum member
- rib cage frame
- spine spacer hardware

Material notes:

- redwood is preferred for visible central members where low mass and stability matter
- harder inserts may be used around high-wear pivots if validated
- grain should generally follow the long axis of vertebral or spine-like members

Primary interfaces:

- head mount
- shoulder girdle
- rib cage
- pelvis
- sensor spine
- soft-tension posture lines

Failure modes:

- cumulative alignment drift
- bushing ovalization
- splitting around close drill holes
- torsional looseness
- unsupported long-grain fracture

Validation needs:

- neutral pose alignment
- vertical stack compression check
- torsion check
- drill spacing review
- end-grain seal review

## 2.3 Pelvic Bowl

Role:

The pelvic bowl transfers load between torso and legs and defines the lower-body stance geometry.

Buildable elements:

- left hip plate
- right hip plate
- sacral lock block
- front bridge
- acetabular socket blocks
- hip spacers
- pelvis hardware stack

Material notes:

- requires stable stock and careful grain orientation
- joint blocks may need denser insert material or reinforced bushing strategy
- visual surfaces should be protected from hardware staining

Primary interfaces:

- axial core
- left femur
- right femur
- soft-tension hip restraints
- sensor routing through pelvis

Failure modes:

- hip socket creep
- splitting across curved grain
- misaligned bilateral joint centers
- seat-load compression damage
- hardware staining at visible surfaces

Validation needs:

- left/right symmetry check
- acetabular center measurement
- bushing retention test
- seated-load simulation or physical load test
- humidity movement coupon

## 2.4 Shoulder Girdle

Role:

The shoulder girdle suspends the upper limbs and creates the upper torso bridge.

Buildable elements:

- left scapula
- right scapula
- left clavicle
- right clavicle
- sternoclavicular interface
- shoulder suspension hardware
- scapular spacer blocks

Material notes:

- flat or curved plates require attention to grain runout
- thin scapular elements are vulnerable to checking and corner damage
- clavicles should preserve long-grain continuity through their length

Primary interfaces:

- axial core
- sternum
- humerus
- soft-tension shoulder straps
- sensor channels for upper limb routing

Failure modes:

- shoulder droop
- scapula plate cracking
- clavicle splitting near holes
- asymmetric shoulder height
- friction washer loosening

Validation needs:

- bilateral height check
- shoulder range check
- suspended-arm load test
- drill tearout test in thin stock

## 2.5 Upper Limbs

Role:

Upper limbs provide gesture, reach, and expressive pose.

Buildable elements:

- humerus
- radius
- ulna
- elbow joint blocks
- wrist interface blocks
- forearm rotation hardware

Material notes:

- long members should keep grain aligned proximal-to-distal
- elbow and wrist blocks need bushing retention tests
- small cross sections require conservative hole spacing

Primary interfaces:

- shoulder joint
- elbow joint
- wrist joint
- soft-tension return lines
- hand wiring or sensor routing

Failure modes:

- split long bone at pin hole
- elbow wobble
- forearm twist misalignment
- wrist interface loosening
- over-thin carved transitions

Validation needs:

- long-grain blank inspection
- hinge axis alignment
- elbow flexion range
- wrist block retention test
- torsion test

## 2.6 Hands

Role:

Hands are high-detail terminal assemblies for expression, touch, and fine articulation.

Buildable elements:

- carpal block set
- metacarpals
- phalanges
- thumb saddle interface
- finger pins
- tendon or cord routing
- fingertip pads or contact surfaces

Material notes:

- small parts amplify grain problems
- dense woods may help wear surfaces but increase splitting risk
- redwood may be too soft for some tiny high-wear contact points unless reinforced

Primary interfaces:

- wrist
- finger tendons
- sensor pockets
- pin and bushing microhardware

Failure modes:

- phalanx splitting
- pin hole breakout
- tendon groove wear
- tiny part loss
- asymmetric finger curl

Validation needs:

- small-hole drilling coupon
- pin retention test
- finger curl range check
- tendon abrasion check
- replaceability review

## 2.7 Lower Limbs

Role:

Lower limbs provide stance, height, pose, and major load paths.

Buildable elements:

- femur
- patella interface
- tibia
- fibula
- knee joint blocks
- ankle interface blocks

Material notes:

- femur and tibia blanks require the cleanest long-grain stock
- knee interfaces should prioritize alignment and bushing retention
- visible curves must not cut across critical grain paths without reinforcement

Primary interfaces:

- pelvis
- knee
- ankle
- soft-tension standing support
- sensor routing for load or position

Failure modes:

- knee axis misalignment
- long-member bowing
- split near high-load pins
- ankle block looseness
- visible checking along carved surfaces

Validation needs:

- full-length straightness check
- grain runout inspection
- knee flexion threshold test
- compression or stance load test
- humidity movement coupon

## 2.8 Feet

Role:

Feet provide ground contact, stance tuning, and stability.

Buildable elements:

- talus interface
- calcaneus block
- tarsal block set
- metatarsals
- toe phalanges
- sole contact pads or foot plates

Material notes:

- feet may require sacrificial sole material or replaceable pads
- contact surfaces need wear and finish testing
- toe parts share the same small-part risks as fingers

Primary interfaces:

- ankle
- ground contact surface
- toe articulation
- possible load sensors

Failure modes:

- foot rock instability
- toe pin breakout
- sole wear
- ankle socket creep
- moisture uptake from ground contact

Validation needs:

- flatness and stance test
- toe articulation test
- sole wear test
- ground moisture isolation review

## 2.9 Joint Hardware System

Role:

The joint hardware system defines motion, serviceability, friction, and replaceability.

Buildable elements:

- pins
- collars
- bushings
- washers
- spacers
- hinge plates
- cam collars
- pawl locks
- limit stops
- friction washers

Material notes:

- copper and brass are project-character materials but require staining tests at wood contact
- stainless may be needed where staining or strength matters
- hardware stack dimensions must be recorded for every joint

Failure modes:

- pin galling
- bushing ovalization
- washer compression into soft wood
- friction drift
- lock misengagement
- staining at visible interfaces

Validation needs:

- hardware stack diagram
- torque/friction check
- retention check
- stain coupon
- serviceability check

## 2.10 Soft Interface / Tension System

Role:

The soft interface system provides restraint, return, damping, and pose feel.

Buildable elements:

- elastic tendons
- cord ligaments
- return springs
- travel limiters
- dampers
- anchor eyelets
- routing grooves
- tension adjusters

Material notes:

- anchors must not split the wood under repeated tension
- grooves should avoid sharp stress risers
- replaceable anchors are preferred over permanent embedded soft goods

Failure modes:

- anchor pullout
- groove abrasion
- elastic fatigue
- tension imbalance
- hidden wear at routing points

Validation needs:

- anchor pull test
- abrasion test
- return-to-neutral test
- range stop test
- service replacement test

## 2.11 Sensor / Signal Routing System

Role:

Sensor routing prepares the armature for future instrumentation without destroying structure.

Buildable elements:

- wire channels
- sensor pockets
- strain relief points
- connector cavities
- service ports
- shielding or grounding paths

Material notes:

- routing must not cut through critical grain paths
- access covers should be serviceable
- sensor pockets require local reinforcement if near joints

Failure modes:

- weakened cross section
- wire pinch
- inaccessible connector
- sensor pocket splitting
- moisture ingress through routing

Validation needs:

- channel clearance check
- bend radius check
- service access check
- structural remaining-section review

## 2.12 Surface / Finish System

Role:

The surface system controls touch quality, moisture moderation, color aging, and hardware interaction.

Buildable elements:

- sanded surfaces
- sealed end grain
- oil, wax, varnish, shellac, or other finish test zones
- burnished edges
- finish transition zones
- hardware contact sealing

Material notes:

- finish must be tested on actual stock before final application
- redwood extractive bleed and metal staining must be checked
- end grain deserves separate treatment from long grain

Failure modes:

- finish adhesion failure
- uneven blotching
- extractive bleed
- hardware stain halo
- raised grain after humidity
- worn high-touch areas

Validation needs:

- finish coupon
- hardware stain coupon
- humidity exposure coupon
- touch/wear coupon

## 2.13 Validation / Test Coupons

Role:

Coupons prevent final components from carrying untested material assumptions.

Required coupon types:

- long-grain strip
- cross-grain strip
- drilled block
- end-grain sealed block
- finish test block
- hardware contact block
- bushing or pin retention block

Failure modes revealed by coupons:

- checking
- splitting
- tearout
- staining
- finish failure
- bushing looseness
- grain instability

Validation result categories:

- pass
- pass with notes
- re-test required
- material not approved for role

Coupons are first-class components and should be tracked with component records when the build matures.
