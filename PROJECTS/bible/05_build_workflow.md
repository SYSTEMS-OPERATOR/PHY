# Part V — Build Workflow and Validation

## 5.1 Purpose

This chapter converts the Wooden Armature Bible from design reference into build procedure.

The workflow is intentionally staged. No final component should be cut before its material, geometry, joint interfaces, and failure risks have been checked.

## 5.2 Build stages

```text
research record
   -> material record
   -> coupon test
   -> prototype component
   -> rough blank
   -> drill schedule
   -> hardware dry fit
   -> refined-aging check
   -> final shaping
   -> validation report
   -> build book inclusion
```

## 5.3 Stage 0: source and record check

Goal:

Confirm that the material and component have enough known data to proceed.

Required inputs:

- material record
- source status
- legal sourcing note
- component guide or draft component guide
- known unknowns

Pass condition:

- all critical missing values are marked
- no hidden assumptions remain
- material is allowed for the intended stage

Fail condition:

- material has no record
- legal status is unclear for final use
- missing values are silently treated as known

## 5.4 Stage 1: material coupon test

Goal:

Test real stock before final cuts.

Minimum coupon set:

- long-grain strip
- cross-grain strip
- drilled block
- end-grain sealed block
- finish test block
- hardware contact block
- bushing or pin retention block

Required observations:

- splitting
- checking
- tearout
- bushing fit
- pin retention
- finish adhesion
- extractive bleed
- staining at metal contact
- dimensional movement after humidity exposure

## 5.5 Stage 2: prototype component

Goal:

Test geometry and assembly logic using DEFAULT material or sacrificial stock.

Prototype does not need final beauty. It must reveal:

- whether the component can be shaped
- whether holes can be drilled cleanly
- whether hardware can be serviced
- whether motion clears neighboring parts
- whether the component can be inspected

Prototype result categories:

- pass to final material
- revise dimensions
- revise joint geometry
- revise hardware
- reject design

## 5.6 Stage 3: rough blank

Goal:

Cut oversize stock with enough allowance for final shaping.

Each rough blank record should include:

- component ID
- material ID
- rough length
- rough width
- rough thickness
- grain direction
- visible defect notes
- fabrication allowance
- intended finished envelope

Rough blank diagram:

```text
ROUGH BLANK

+--------------------------------+
|                                |
|   finished envelope inside     |
|      +----------------+        |
|      |                |        |
|      +----------------+        |
|                                |
+--------------------------------+

outer: rough blank
inner: finished part envelope
```

## 5.7 Stage 4: drill schedule

Goal:

Define all holes before irreversible cutting.

A drill schedule should include:

- component ID
- hole ID
- hole diameter
- hole depth
- through-hole or blind-hole
- axis direction
- distance from reference edges
- hardware item
- clearance or interference fit
- counterbore or countersink if any
- pilot-hole requirement

Drill risk checks:

- edge distance
- hole spacing
- grain direction
- exit tearout
- bushing wall thickness
- hardware service access

## 5.8 Stage 5: hardware dry fit

Goal:

Assemble hardware before final finishing.

Check:

- pin fit
- bushing retention
- washer compression
- collar alignment
- friction tuning
- lock engagement
- service removal
- staining risk

Hardware stack diagram:

```text
HARDWARE STACK

nut | washer | side plate | bushing | component | bushing | side plate | washer | nut
```

Every joint should have a stack order recorded.

## 5.9 Stage 6: refined-aging check

Goal:

Confirm that finishing, sealing, humidity movement, and hardware contact behave acceptably.

Minimum checks:

- end-grain seal inspection
- finish adhesion check
- metal staining check
- humidity movement check
- bushing fit after humidity exposure
- raised grain check
- touch/wear surface check

## 5.10 Stage 7: final shaping

Goal:

Shape the component to final envelope after irreversible tests have passed.

Rules:

- preserve grain continuity through load paths
- avoid sharp internal corners
- avoid over-thinning around holes
- mark all hardware centers before sculptural refinement
- protect reference faces until final fitting
- keep service access visible and reachable

## 5.11 Stage 8: validation report

A validated component should report:

- component ID
- material ID
- finished dimensions
- mass if known
- joint centers
- hardware assignments
- drill schedule refs
- coupon test refs
- finish system
- validation status
- open unknowns

Validation states:

| State | Meaning |
|---|---|
| missing | no usable record exists |
| incomplete | record exists but required data is missing |
| draft | suitable for prototype work only |
| tested | coupon or prototype tests exist |
| ready | approved for final cut under current assumptions |
| rejected | not suitable for intended role |

## 5.12 Stage 9: build book inclusion

A component can enter the build book only after it has:

- a component guide
- a material record
- rough blank dimensions
- finished envelope dimensions
- grain direction
- joint centers
- hardware assignments
- drill schedule
- validation status
- unknowns listed

## 5.13 Failure handling

Failure is useful if recorded.

Failure records should include:

- component or coupon ID
- material ID
- observed failure
- test condition
- likely cause
- corrective action
- retest requirement

Common failures:

- split near hole
- tearout
- bushing looseness
- finish bleed
- hardware staining
- grain runout
- over-flexing
- stop surface crushing
- humidity movement beyond tolerance

## 5.14 Offline review packet

A complete offline review packet should include:

- material records
- component guides
- coupon results
- drill schedules
- hardware cut lists
- validation reports
- build book chapter
- diagrams
- open unknowns

## 5.15 Minimum viable first build set

Recommended first component set for REDWOOD:

- femur test component
- humerus test component
- rib template
- phalanx template
- scapula plate
- pelvis socket block
- hinge coupon
- bushing retention coupon
- finish and hardware stain coupon

This gives coverage across long members, flat plates, small members, joint blocks, and surface/finish behavior.
