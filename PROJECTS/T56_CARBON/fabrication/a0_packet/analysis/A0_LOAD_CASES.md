# T56 A0 load-case calculation report

Status: **CLOSED-FORM ANALYSIS COMPLETE — PHYSICAL VALIDATION OPEN**

These are deterministic design checks, not measured capability, FEA, or test evidence.

| Case | Key calculated result | Physical disposition |
| --- | --- | --- |
| LC-SHO-001 | service_gravity_moment_Nm = 5.44024 | test record open |
| LC-SHO-002 | ordinary_stop_energy_J = 0.00688741 | test record open |
| LC-SHO-003 | retention_design_moment_Nm = 95.1 | test record open |
| LC-SHO-004 | shaft_yield_margin_x = 3.47431 | test record open |
| LC-SHO-005 | degraded_single_fault_catch_energy_J = 1.40804 | test record open |
| LC-MNT-001 | retention_handling_force_N = 300 | test record open |

## Method limits

- closed-form rigid-joint checks do not replace FEA, coupon data, or article test
- nominal mass is conservatively represented at half-span and payload at the load station
- linear stop force assumes full usable pad travel and triangular force-displacement response
- bearing load sharing, friction, damping, backlash, acoustic output, temperature and fatigue remain physical measurements
- no calculated result is a measured capability

Numeric acceptance criteria are recorded in `load_case_register.json`. Passing the arithmetic does not approve a load case; approval requires the corresponding signed bench record.
