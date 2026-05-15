# Part IV — Materials and Refined Aging

## 4.1 Material truth hierarchy

Material selection for the wooden armature is governed by local project records.

Priority order:

1. verified source-backed material record
2. partial source-backed record with explicit unknowns
3. local coupon test result
4. prototype-only assumption
5. missing-data record

No material property should be promoted to fabrication truth without a source or test.

## 4.2 Age versus material property

Tree age is not a material property.

Age can indicate that a tree survived long-duration stress, but it does not directly provide:

- hardness
- stiffness
- rupture strength
- crushing strength
- shrinkage behavior
- fastener behavior
- finish compatibility

The correct model is:

```text
extreme age
  -> evidence of survival traits
  -> possible design relevance
  -> requires verified material values and coupon tests
```

## 4.3 Refined-aging model

After wood is refined into armature components, its endurance is governed by:

- moisture exchange
- dimensional movement
- end-grain sealing
- finish behavior
- UV exposure
- biological decay risk
- extractive bleed
- hardware staining
- drill and bushing stress concentrations
- grain continuity through small parts
- storage and service environment

## 4.4 Moisture and dimensional movement

Wood remains hygroscopic after cutting and finishing.

This means every component must be designed with moisture movement in mind.

Important consequences:

- end grain exchanges moisture rapidly
- thin components may move differently than thick blocks
- holes can loosen or tighten with humidity change
- finish slows moisture exchange but does not eliminate it
- cross-grain assemblies need room to move

Elementary diagram:

```text
END GRAIN MOISTURE PATH

  long grain face        end grain
  ---------------        ||||||||
  slow exchange          fast exchange

 seal priority: end grain first
```

## 4.5 Radial and tangential shrinkage

Wood does not shrink equally in all directions.

```text
LOG SECTION

        tangential direction
       <------------------>
        ________________
      /                  \
     |        pith        |
      \__________________/
              ^
              |
          radial direction
```

Tangential shrinkage is usually greater than radial shrinkage.

Design consequence:

- flat plates may cup
- round sections may distort
- bushings near growth-ring transitions may loosen unevenly
- grain orientation must be recorded for critical blanks

## 4.6 Finish and surface aging

Surface finish is not cosmetic only. It is part of the aging system.

Finish functions:

- slows moisture exchange
- changes touch feel
- affects color aging
- reduces staining or bleed when compatible
- may fail if applied after UV-altered surface chemistry

Required finish tests:

- adhesion
- blotching
- extractive bleed
- raised grain after humidity
- high-touch wear
- hardware-contact staining

## 4.7 Hardware staining and contact risk

Metal contact is a material interface, not just assembly detail.

Risk factors:

- tannin or extractive content
- reactive metals
- moisture
- finish discontinuities around holes
- washer compression into soft wood

Minimum hardware-contact coupon:

```text
HARDWARE STAIN COUPON

[wood block]
  O  brass washer
  O  copper washer
  O  stainless washer
  O  test steel washer

observe: halo, bleed, corrosion, compression
```

## 4.8 Current material families

### REDWOOD

Primary build candidate.

Strengths:

- low density
- low shrinkage
- stable carved behavior
- decay-oriented material profile
- strong visible character

Risks:

- softness at high-wear joints
- extractive bleed
- staining near reactive metal
- unknown fastener behavior in current local record

### CYPRESS

Water-endurance challenger.

Strengths:

- decay-resistance comparator
- good nailing, gluing, finishing, and paint-holding behavior in current local record
- useful damp-risk candidate

Risks:

- tearout risk requiring sharp tools and light passes
- raw unfinished surfaces may feel greasy
- source distinction between old-growth and younger material matters

### JUNIPER

Arid endurance challenger.

Strengths:

- durable and termite-resistant in current local record
- harder than redwood and cypress in current matrix
- useful for small dense detail parts if stock is clean

Risks:

- knots
- grain irregularity
- incomplete radial/tangential shrinkage split
- possible checking around holes

### DEFAULT

Prototype and control family.

Roles:

- Douglas fir: strength/stiffness baseline
- hard maple: dense/hard wear control
- yellow poplar: easy-carve draft stock

DEFAULT parts are not final material recommendations unless separately validated.

### BRISTLECONE

Endurance reference only.

Current support:

- species-level density and specific-gravity anchor
- academic endurance and age-growth references

Current gaps:

- no local Janka hardness
- no local MOR
- no local MOE
- no local crushing strength
- no local shrinkage values
- no fabrication authorization

## 4.9 Sample-test gate

Before final cuts, each candidate stock type should pass the sample-test gate.

Required coupons:

- long-grain strip
- cross-grain strip
- drilled block
- end-grain sealed block
- finish test block
- hardware contact block
- bushing or pin retention block

Pass condition:

- no uncontrolled splitting around holes
- acceptable tearout under planned bit and feed conditions
- stable bushing or pin fit after humidity exposure
- acceptable surface finish behavior
- no unacceptable staining at planned hardware interfaces
- no unresolved unknown for intended component role

Fail condition:

- checking crosses a critical load or visual surface
- bushing fit loosens after humidity exposure
- finish fails adhesion or bleeds badly
- hardware contact stains beyond acceptable visual limits
- grain runout makes the part unsafe or unstable
- stock cannot be legally or repeatably sourced

## 4.10 Practical material conclusion

Current build logic:

```text
final visible primary stock: coast redwood
endurance challengers: bald cypress, western juniper
prototype baselines: Douglas fir, hard maple, yellow poplar
age-reference control: Great Basin bristlecone pine
```

The armature should be built from verified, testable, repeatably sourced material.

Ancient endurance is valuable as design philosophy, but it must not override fabrication truth.
