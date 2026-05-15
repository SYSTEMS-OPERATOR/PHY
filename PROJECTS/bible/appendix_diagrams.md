# Appendix C — Diagram Library

## C.1 Purpose

This appendix stores reusable elementary diagrams for the Wooden Armature Bible.

The diagrams are intentionally plain text so they remain useful offline and can later be redrawn as SVG or technical figures.

## C.2 Grain direction

```text
LONG MEMBER GRAIN

proximal                                      distal
  |                                             |
  v                                             v
+------------------------------------------------+
| >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> |
+------------------------------------------------+

preferred: grain follows the long load path
avoid: severe runout through pin holes
```

## C.3 End-grain sealing

```text
END-GRAIN SEALING

long grain face                 end grain
+--------------------+          ||||||||||
|                    |          ||||||||||  fast moisture path
+--------------------+          ||||||||||

seal priority: end grain first
```

## C.4 Hinge arc

```text
HINGE ARC

       rotation axis
            v
[A]---------O---------[B]
             \
              \
               \ allowed arc
                X hard stop
```

## C.5 Soft stop and hard stop

```text
MOTION THRESHOLD

neutral ---- free motion ---- soft stop zone ---- hard stop
   |             |                  |                 |
   0            low              rising              max
```

## C.6 Ball-and-socket envelope

```text
BALL SOCKET ENVELOPE

       socket rim / hard limit
          ___________
        /             \
       /     ( O )     \
       \               /
        \_____________/
              |
              stem

DOF: 3 rotational
risk: pullout, cup cracking, uncontrolled flop
```

## C.7 Bushing cross section

```text
BUSHING CROSS SECTION

side plate | washer | wood | bushing | pin | bushing | wood | washer | side plate
    |          |       |       |       |      |        |       |          |
   [ ]--------[ ]-----[W]-----(B)-----(P)----(B)------[W]-----[ ]--------[ ]
```

## C.8 Washer compression

```text
WASHER COMPRESSION RISK

small washer:             larger washer:

   force                     force
    v                         v
   [ ]                       [     ]
    |                         |
  crushed zone              distributed load
```

## C.9 Drilled-hole edge distance

```text
EDGE DISTANCE

+------------------------+
|                        |
|   O hole               |
|<->                     |
| edge distance          |
+------------------------+

rule: edge distance must be validated per material, hole size, and load role
```

## C.10 Sliding slot

```text
SLIDING SLOT

+-------------------------+
|   O  --->               |
|   pin travel            |
+-------------------------+

risk: tearout at slot ends
need: washer coverage and reinforced ends
```

## C.11 Tendon anchor

```text
TENDON ANCHOR

elastic line
    |
    v
----O==== routed groove ====O----
   anchor                  anchor

risk: pullout, groove abrasion, sharp-corner cutting
```

## C.12 Sensor pocket

```text
SENSOR POCKET

+------------------------+
|                        |
|   [ sensor pocket ]    |
|        | wire channel  |
|        v               |
+------------------------+

risk: weakened cross section
need: service access and strain relief
```

## C.13 Hardware stain coupon

```text
HARDWARE STAIN COUPON

+------------------------+
| O brass                |
| O copper               |
| O stainless            |
| O test steel           |
+------------------------+

observe: halo, bleed, corrosion, compression mark
```

## C.14 Humidity coupon

```text
HUMIDITY MOVEMENT COUPON

before:  +------------+
after:   +----------- )

observe:
- cup
- twist
- check
- hole fit change
- bushing looseness
```

## C.15 Component envelope

```text
ROUGH BLANK AND FINISHED ENVELOPE

+--------------------------------+
|                                |
|   +------------------------+   |
|   | finished envelope      |   |
|   +------------------------+   |
|                                |
+--------------------------------+

outer: rough blank
inner: finished component envelope
```

## C.16 Diagram backlog

Future diagrams should include:

- full-body primary component map
- shoulder suspension joint
- hip socket threshold map
- knee hinge stack
- elbow hinge stack
- wrist compound joint
- finger cascade
- thumb saddle-like joint
- ankle limited universal
- pelvis bowl exploded view
- scapula/clavicle suspension map
