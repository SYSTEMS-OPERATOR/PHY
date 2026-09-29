# T56 A0-R1 discrepancy register

Status: **AUDIT COMPLETE — CORRECTIONS PLANNED — NOT FABRICATION RELEASED**

Scope is limited to the left thoracic-root-to-dummy-humerus A0 bench article.
No item below changes SOPHY canon 1.0.0 or makes an anatomical claim.

| ID | Severity | Confirmed discrepancy and source | A0-R1 disposition | Required regression evidence |
| --- | --- | --- | --- | --- |
| R1-001 | critical | `build_parts()` extrudes the yoke `8 mm` with `both=True`, producing a nominal 16 mm part; the BOM and mechanism decision specify 8 mm. The checked manifest reports 16.01 mm. | Generate one 8 mm side plate and instantiate two plates in the assembly. | Part bound is 8 mm; assembly contains two correctly spaced instances. |
| R1-002 | critical | The index sector is extruded `6 mm` with `both=True`, producing 12 mm; the BOM and mechanism decision specify 6 mm. The checked manifest reports 12.03 mm. | Generate a 6 mm sector. | Part bound is 6 mm and drawing/BOM agree. |
| R1-003 | critical | The sector center opening is modeled Ø12 while the controlled shaft is Ø17 h6. | Change the sector to a keyed Ø17 shaft interface; keep vendor-dependent plunger details open. | Nominal shaft passes the bore and key engagement is represented. |
| R1-004 | critical | The output hub contains a Ø17 shaft opening and 25.5 mm tube socket whose volumes intersect; the model has no controlled tube insertion stop. | Separate the shaft hub from a coaxial tube clamp with a 30 mm insertion shoulder; document the load path. | No unintended void intersection; insertion depth and S/E closure are tested. |
| R1-005 | major | README calls the removable actuator interface “8 mm keyed,” but the mechanism decision and BOM control a 17 mm shaft with a 5 × 5 mm key. | Correct all packet prose and drawings to the 17 mm keyed shaft. | Text scan finds no obsolete 8 mm drive-interface claim. |
| R1-006 | critical | CAD omits the 5 × 5 keyway/key, M6 hub pinch holes, shaft end capture, bearing axial retention and friction-stack interfaces described by BOM/procedure. | Add simplified but dimensioned interface geometry and purchased-component envelopes. | Feature and assembly-instance tests; reviewer drawing callouts. |
| R1-007 | critical | Root pivots are holes through the 8 mm base while P/A are specified 35 mm above its top; no standoffs realize that datum. | Add two root standoffs that locate the pivot axes at Z=35 mm. | Located pivot axes match P/A within numerical tolerance. |
| R1-008 | critical | The checked CAD contains no located assembly, so four-bar closure, S/E realization, quantities, tool access and interference are unverified. | Export a neutral located assembly plus a machine-readable instance/clearance report. | Transform, quantity, datum, closure and sampled-motion tests. |
| R1-009 | major | Stop pads exist as loose solids; their mounts/contact geometry at ±10 and -32/+92 degrees are undefined. Plunger and tether are prose-only. | Add review-level stop brackets, contact definitions, plunger envelope and tether endpoints. | Required instances present; motion report identifies intentional contacts and clearance limits. |
| R1-010 | major | CAD and BOM repeat literals instead of consuming the governing requirement JSON; calculation sections repeat some of the same dimensions separately. | Add one `fabrication_geometry` block to the project-local mechanism decision and make exporter/calculations consume it. | Parameter-drift tests compare CAD, BOM, drawings and analysis inputs to the authority. |
| R1-011 | critical | Existing tests establish valid individual solids but not assembly compatibility or interface strength. | Add dimensional, feature, located-assembly, sampled-clearance and load-path checks. | Focused tests fail for the A0-R0 defects above and pass only after correction. |
| R1-012 | critical | Shaft analysis is pure torsion only; key, pin, bearing, yoke, stop attachment, fastener/root and combined shaft loading are not checked. | Extend conservative closed-form checks without converting calculations into evidence. | Six cases retain numeric criteria; unsupported supplier/physical claims remain open. |

## Bounded correction plan

1. Add project-local fabrication dimensions and interface definitions to
   `requirements/a0_shoulder_mechanism.json`; preserve the selected kinematics,
   frozen mission envelope and all canon-bound files.
2. Rebuild only the A0 article parts and located assembly from those values.
   Export deterministic part/assembly review artifacts and updated drawings.
3. Extend closed-form load-path checks and focused tests. Keep supplier
   verification, independent review and every physical record explicitly open.

## Audit boundary

The audit confirms internal inconsistencies; it does not validate the proposed
corrections, supplier hardware, manufacturing process, structural safety or
physical performance. Those claims remain blocked by review and measurement.
