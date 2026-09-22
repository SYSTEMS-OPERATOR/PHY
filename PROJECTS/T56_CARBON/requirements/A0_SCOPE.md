# T56 A0 single-side shoulder article scope

Status: **design requirements frozen; physical qualification open**

This packet is limited to the existing
`single_side_thoracic_root_to_dummy_humerus` convergence article. It is a
guarded, supervised bench prototype with umbilical power. It is not a complete
arm, body-weight support, human-lifting device, production mechanism, or proof
of whole-body geometry.

The machine-readable authority is `a0_article_scope.json`. Its ten article
mission inputs are project-local requirements chosen to bound a useful, low-risk
A0 test. They are not anthropometric laws, measured performance, material
allowables, or evidence that an article passed. The requirements may be revised
by a later project decision without changing SOPHY identity canon.

## Frozen envelope

| Requirement | A0 design value |
| --- | ---: |
| Outboard arm assembly mass | 2.5 kg maximum |
| Hand-equivalent payload | 0.5 kg maximum |
| Shoulder speed | 20 deg/s maximum |
| Shoulder acceleration/deceleration | 40 deg/s² maximum |
| Accidental push/pull | 100 N |
| Minor guarded impact | 2 J |
| Demonstration life | 10,000 cycles, followed by inspection |
| Operational sound target | 50 dBA at 1 m, no payload, 20 deg/s |
| Supported maintenance force | 100 N |

Safety factors are 2.0 on yield, 3.0 on ultimate, buckling and joint
retention, and 4.0 on bonded/keyed inserts. These are sizing requirements, not
permission to use unknown allowables. Bonded or composite structure still needs
representative coupons; purchased hardware still needs manufacturer data or
proof testing.

## Release boundary

The A0 packet may become **shop-reviewable** after drawings, calculations and
inspection/test procedures are complete. It remains **not fabrication
qualified** until the selected stock and interfaces have traceable properties,
the fixture has been measured, and the article passes static, retention,
clearance, impact and cyclic tests. No unmeasured result may be recorded as
approved evidence.

Canon `1.0.0` remains sovereign at `H = span = 1676.4 mm`. This scope file may
select materials, mechanisms and test limits; it may not bind new identity
landmarks or change the mirror law.
