# T56 A0 single-side shoulder fabrication packet

Status: **SHOP REVIEW CANDIDATE — NOT FABRICATION RELEASED**

This packet covers only the left-side thoracic-root-to-dummy-humerus bench
article. It implements the scope frozen in `../../requirements/A0_SCOPE.md` and
the project-local mechanism decision in
`../../requirements/a0_shoulder_mechanism.json`. It does not model the full
body, revise SOPHY canon 1.0.0, or claim physical qualification.

## Selected mechanism

The scapular stage is a 55 mm planar parallel four-bar with 100 mm root and
coupler spacing and ±10 degree physical stops. The moving carriage remains
parallel to the fixture. The cartridge is one pitch axis through the neutral
point `S=(-205,-10,70) mm`, with a commanded range of -30 to +90 degrees and
independent mechanical stops at -32 and +92 degrees.

Retention does not depend on power or software. The carriage uses a clamp and
spring-biased neutral index. The pitch stage uses a Belleville-preloaded dry
friction stack set to at least 7 N·m plus an 8 mm spring-biased sector plunger
at 15 degree intervals; a 6 mm, at-least-1 kN steel tether is secondary
retention, not the primary retainer. A removable low-speed actuator may use the
8 mm keyed drive interface. Its selection is outside the A0 fabricated load
path and must not alter the stops or passive retention.

## Drawing register

| Drawing | Title | Controlling content |
| --- | --- | --- |
| A0-D001-R0 | Fixture datum and root interface | frame realization, base plate, mounting pattern, roots P/A |
| A0-D002-R0 | Four-bar and pitch cartridge | pivot pattern, center S, ranges, stops and retention |
| A0-D003-R0 | Dummy humeral member | 327 mm tube cut, section, S/E reference stack |
| A0-D004-R0 | Cartridge interfaces | shaft/bearing fits, hub socket, index sector |

SVGs are review drawings and must be read at their stated dimensions; do not
scale the graphic. STEP files are neutral part solids. STL files are visual
review meshes, not machining authority. `manifest.json` records solid and STEP
round-trip checks. `BOM.csv` uses packet item IDs and vendor-neutral procurement
specifications; the shop must record supplier lots and confirm actual bearing
and plunger dimensions before machining mating features.

## Shop-review boundary

Before cutting stock, require an independent drawing/load-path review, shop
process-capability confirmation, procurement cross-check, and disposition of
every substitution. As-built inspection, proof, cycling, impact and acoustic
records remain open even after analytical validation. No unmeasured property is
represented as evidence in this packet.

Regenerate the packet from the repository root with:

```sh
python bin/export_t56_a0_article.py
```

Closed-form calculation sheets and numeric acceptance criteria are in
`analysis/`; controlled build, inspection and bench-test steps plus deliberately
blank record templates are in `procedures/`. The checked-in records do not claim
that any physical test has run.
