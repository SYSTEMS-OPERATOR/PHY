# T-5.6 Convergence Requirements

Status: REQUIREMENTS_OPEN_NOT_SIMULATION_READY

This package turns the architecture established in T-5.6 pull requests 85 through 88 into a deterministic convergence gate. It does not add another body subsystem and it does not infer fabrication values from concept imagery.

## Purpose

The registers answer four questions in machine-readable form:

1. Which project decisions are already locked?
2. Which mission inputs must be supplied before structural loads can be calculated?
3. Which geometry must be dimensioned before the thorax and shoulder can be simulated?
4. Which load cases must be approved before a single-side test article can be released?

## Files

- mission_envelope.json records locked project decisions, open operating inputs, and conservative scope exclusions.
- geometry_register.json defines the shared datum and every geometry item needed by the trunk-to-left-shoulder convergence article.
- load_case_register.json defines the minimum structural, contact, maintenance, and power-loss cases.
- single_side_article_closure.json selects the exact dependency union for the next single-side kinematic and structural article.
- ../tools/check_convergence.py audits identifiers, references, locked values, and readiness blockers without external dependencies.
- geometry_contracts.md defines typed dimension, datum, path, joint, and envelope inputs; ../tools/geometry_contracts.py validates them without supplying measurements.

## Gate sequence

### Gate G0 — mission envelope

Close every required input in mission_envelope.json with a value, unit, authority, and evidence reference.

### Gate G1 — simulation geometry

Dimension and lock every geometry_register.json parameter marked required_for_simulation. Structural geometry and outer-form geometry remain separate.

Supplied geometry must satisfy the [typed input contract](geometry_contracts.md),
including finite numeric values, exact units, explicit frames, and positive
engineering tolerances. The local fixture datum must also be dimensioned with a
revision and drawing evidence. Both scopes verify scale/span against the executable
canon; repeated matching profile numbers are not sufficient authority.

### Gate G2 — load cases

Approve every load case with numeric inputs, boundary conditions, safety factors, and pass/fail criteria. A qualitative load name is not an approved load case.

### Gate G3 — test drawing release

Select the mechanism and material systems, produce coupons, and release only the single-side thoracic-root-to-dummy-humerus article. Paired integration follows evidence from that article.

## Audit

Run from the repository root:

    python3 PROJECTS/T56_CARBON/tools/check_convergence.py

The default audit returns success when the files are internally valid and reports every full-system blocker. Strict mode returns a non-zero status until the package is simulation-ready:

    python3 PROJECTS/T56_CARBON/tools/check_convergence.py --strict

Audit the next article without weakening the full-system gate:

    python3 PROJECTS/T56_CARBON/tools/check_convergence.py --profile single-side
    python3 PROJECTS/T56_CARBON/tools/check_convergence.py --profile single-side --strict

The single-side profile is the exact dependency union of the six canonical load cases assigned to the thoracic-root-to-dummy-humerus article. It reduces the active closure surface from 13 mission inputs, 19 geometry parameters, and 9 load cases to:

- 10 mission inputs;
- 11 geometry parameters;
- 6 load cases;
- 5 evidence packages covering the article fixture, mechanism and retention, material and joint stack, kinematic/collision/service model, and analysis/test record.

The profile supplies no values. Every numeric closure still requires the authority, evidence, tolerance, acceptance criteria, and approval evidence required by the source registers.

## Software stabilization evidence

`tests/test_t56_geometry_contracts.py` exercises malformed geometry and gate-bypass
regressions, plus synthetic positive controls. This stabilizes input validation;
it does not close physical geometry or article evidence. See the contract's
verification section for offline commands and remaining validation limits.

## Change rule

A convergence PR must do at least one of the following:

- close a required input with traceable evidence;
- lock dimensioned geometry;
- approve a numeric load case;
- record a mechanism or material decision;
- add test evidence.

Topology-only expansion does not advance these gates.
