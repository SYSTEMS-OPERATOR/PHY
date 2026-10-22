# T56 A0-R2 cartridge engineering packet

**SHOP REVIEW CANDIDATE — NOT FABRICATION RELEASED.** No physical evidence,
simulation qualification or independent approval is claimed. The frozen A0
mission, 317 mm station, 327 mm tube and anatomical canon remain unchanged.

R2 replaces interfering yoke plates with handed footed housings, raises and
relieves the carriage plate, opens the tube socket in its actual insertion
direction, and models retaining C caps, shaft-end bolts/washers, inner-ring
spacers and real pitch-stop contacts. The stop lug is integral with the hub;
shouldered steel crossbars route stop loads into two bolted carrier frames.

`manifest.json` is the current drawing/STEP/BOM register. Fourteen sheets contain
CAD orthographic projections, cut sections, dimension lines, feature coordinates
and candidate tolerances. `cad/A0-R2-parts-step.tar.gz` contains all part models;
`assembly/A0-R2-neutral-assembly.step.gz` contains the issued subassembly. Tapped
holes and screw threads use tap-drill/root envelopes with explicit thread
callouts; bearing internals and supplier mechanisms are not modeled.

`assembly/interface_check_report.json` distinguishes exact nominal and finite
pose checks from unresolved whole-article interfaces. Undefined legacy scapular
stops/plungers and the straight tether envelope are **excluded** from the issued
STEP, with their original collisions retained in that report. Their BOM entries
are reserved requirements. They are not accepted installed hardware. The friction
envelope also lacks a qualified grounded reaction/coupling. Full rocker/bolt
motion, guarding and tool access are still open.

The corrected load report separates shaft torque from bearing bending couples
and includes the rotating member's inertia resultant. SKF 6203-2RSH is a catalog
candidate, with exact abutment data and primary-source provenance in
`../../requirements/a0_supplier_basis.json`. A static screen is conditional on
an unverified force/displacement curve; h6/CN fits and delivered internal
clearance remain blocked. Neither a favorable catalog ratio nor a CAD contact
approves a bearing or a stop.

Most critically, the old assumed **4 mm stop travel** is not established by
acetal thickness. Nominal first contact at -32/+92 leaves no credited compression
within those limits. Four millimeters at R60 would overrun by about 3.82°.
`procedures/A0_STOP_COUPON_REVIEW.md` specifies the minimum resolving evidence.
No impact test or stock cutting is authorized while this is unresolved.

See `analysis/A0_R2_DEVELOPMENT.md`, `analysis/A0_R2_ENGINEERING.md`, the interval
stacks, discrepancy register and independent checklist. R1 discrepancy/validation
files are historical baseline records; they do not describe current geometry.
All six physical load cases and five evidence packages remain unapproved.

Regenerate from the repository root:

```sh
python bin/export_t56_a0_article.py
python PROJECTS/T56_CARBON/tools/a0_load_cases.py --output-dir PROJECTS/T56_CARBON/fabrication/a0_packet/analysis
python PROJECTS/T56_CARBON/tools/a0_convergence_report.py --output-dir PROJECTS/T56_CARBON/fabrication/a0_packet/analysis
```
