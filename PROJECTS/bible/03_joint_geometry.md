# Part III — Joint Geometry and Degrees of Freedom

## 3.1 Purpose

This chapter defines elementary joint geometry for the wooden armature.

The diagrams are intentionally simple and text-first. They are not final manufacturing drawings. They define motion logic, DOF, threshold vocabulary, and validation expectations.

## 3.2 DOF vocabulary

| Term | Meaning |
|---|---|
| 1-DOF | one independent motion axis |
| 2-DOF | two independent motion axes |
| 3-DOF | three independent rotational axes |
| flexion | closing angle, usually forward bending |
| extension | opening angle, usually straightening or backward bending |
| abduction | motion away from body midline |
| adduction | motion toward body midline |
| internal rotation | rotation toward inward orientation |
| external rotation | rotation toward outward orientation |
| hard stop | rigid physical limit |
| soft stop | compliant or frictional limit |
| neutral | intended zero/rest pose |

## 3.3 Joint threshold record

Every articulated joint should eventually define:

```json
{
  "joint_id": "",
  "joint_type": "",
  "dof_count": null,
  "neutral_angle_deg": null,
  "min_angle_deg": null,
  "max_angle_deg": null,
  "soft_stop_start_deg": null,
  "hard_stop_deg": null,
  "friction_adjustable": false,
  "lockable": false,
  "validation_status": "missing | draft | tested | approved"
}
```

## 3.4 Hinge joint

```text
HINGE JOINT

       rotation axis
            |
            v
 [bone A]---O---[bone B]
             )
            )  allowed arc
           )

DOF: 1 rotational
primary motion: flexion / extension
common uses: elbow, knee, finger joints, toe joints
```

Thresholds:

- neutral angle
- flexion maximum
- extension maximum
- hard-stop angle
- soft-stop onset

Failure modes:

- off-axis drilling
- bushing looseness
- pin-hole splitting
- side loading
- stop surface crushing

## 3.5 Pin joint

```text
PIN JOINT

 top view

   washer   bone   bushing   bone   washer
     |       |        |       |       |
    [ ]-----[A]------(O)-----[B]-----[ ]
                      ^
                      pin axis

DOF: usually 1 rotational
```

Purpose:

A pin joint is the simplest buildable rotational connection. It may behave as a hinge when constrained by side plates or geometry.

Validation:

- pin diameter fit
- washer stack clearance
- bushing retention
- wood compression around washer
- service removal

## 3.6 Ball-and-socket joint

```text
BALL AND SOCKET

        socket cup
       /        \
      /   (O)    \
      \          /
       \________/
          |
          stem

DOF: 3 rotational
motions: flexion/extension, abduction/adduction, rotation
common uses: shoulder, hip
```

Thresholds:

- socket rim defines hard-stop envelope
- soft stop may be provided by tension lines
- rotation may need friction collar or damper

Failure modes:

- socket wall cracking
- ball pullout
- uncontrolled flop
- uneven friction
- wear at cup rim

## 3.7 Limited universal joint

```text
LIMITED UNIVERSAL

      y-axis fork
        \   /
         \ /
          O---- x-axis fork
         / \
        /   \

DOF: 2 rotational
motions: flexion/extension + side tilt
```

Purpose:

Useful where a ball joint is too free but a hinge is too limited.

Validation:

- both axes clear through full intended range
- no diagonal binding
- hard stops are symmetric
- fasteners remain serviceable

## 3.8 Saddle-like joint

```text
SADDLE-LIKE JOINT

    concave A:  (____)
    convex B:    /\
                /  \

DOF: 2 coupled rotational
common use: thumb-like opposition concept
```

Purpose:

A saddle-like joint creates controlled compound motion without full ball-joint freedom.

Risks:

- difficult wood shaping
- uneven wear
- ambiguous neutral pose
- complex threshold validation

## 3.9 Sliding slot

```text
SLIDING SLOT

 [bone A]  -----------
           |   O ---> |
           -----------
              pin travels

DOF: 1 translational, sometimes coupled with rotation
```

Purpose:

A slot allows limited translation for adjustment, compliance, or anatomical approximation.

Validation:

- slot end reinforcement
- tearout at slot ends
- washer coverage
- sliding friction
- debris tolerance

## 3.10 Cam collar friction joint

```text
CAM COLLAR

       lever / cam
          |
          v
      ___/ \___
     |         |
 [A]-|   O     |-[B]
     |_________|

DOF: depends on inner joint
function: adjustable friction clamp
```

Purpose:

A cam collar lets the joint be tuned between free motion and held pose.

Validation:

- repeatable clamp force
- no crushing of wood fibers
- accessible adjustment
- no accidental over-locking

## 3.11 Pawl lock

```text
PAWL LOCK

       pawl
        v
       /|
      / |
  ___/  |____ ratchet arc
      ^ ^ ^

DOF: joint motion plus discrete lock positions
```

Purpose:

A pawl lock creates repeatable held positions.

Risks:

- tooth wear
- noisy engagement
- accidental lock
- poor release access

## 3.12 Elastic return joint

```text
ELASTIC RETURN

 [A]---O---[B]
       |
      / \
 elastic return line

DOF: defined by main joint
function: return-to-neutral force
```

Purpose:

Elastic returns prevent loose hanging motion and help the armature feel intentional in pose.

Validation:

- return-to-neutral accuracy
- elastic fatigue
- anchor pullout
- abrasion at routing points

## 3.13 Hard stop vs soft stop

```text
STOP ARC

        hard stop
           |
           v
 neutral ---)---- soft stop zone ----X
             \
              \ allowed motion
```

Hard stop:

- physical boundary
- protects joint from overtravel
- may concentrate force

Soft stop:

- elastic, frictional, or damped resistance
- improves motion feel
- should not be sole safety stop where damage is likely

## 3.14 Joint validation checklist

Every joint should be checked for:

- axis alignment
- intended DOF count
- neutral pose
- min/max angle
- stop type
- friction behavior
- bushing retention
- hardware stack order
- service access
- wood splitting risk
- repeatability after humidity exposure

## 3.15 Diagram expansion backlog

Future graphic diagrams should include:

- full hip ball-and-socket threshold map
- shoulder suspension joint
- elbow hinge stack
- knee hinge stack
- wrist compound joint
- ankle limited universal joint
- thumb saddle-like joint
- finger hinge cascade
- bushing cross section
- washer/collar compression map
