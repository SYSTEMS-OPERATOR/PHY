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
- ../tools/check_convergence.py audits identifiers, references, locked values, and readiness blockers without external dependencies.

## Gate sequence

### Gate G0 — mission envelope

Close every required input in mission_envelope.json with a value, unit, authority, and evidence reference.

### Gate G1 — simulation geometry

Dimension and lock every geometry_register.json parameter marked required_for_simulation. Structural geometry and outer-form geometry remain separate.

### Gate G2 — load cases

Approve every load case with numeric inputs, boundary conditions, safety factors, and pass/fail criteria. A qualitative load name is not an approved load case.

### Gate G3 — test drawing release

Select the mechanism and material systems, produce coupons, and release only the single-side thoracic-root-to-dummy-humerus article. Paired integration follows evidence from that article.

## Audit

Run from the repository root:

    python3 PROJECTS/T56_CARBON/tools/check_convergence.py

The default audit returns success when the files are internally valid and reports every remaining blocker. Strict mode returns a non-zero status until the package is simulation-ready:

    python3 PROJECTS/T56_CARBON/tools/check_convergence.py --strict

## Change rule

A convergence PR must do at least one of the following:

- close a required input with traceable evidence;
- lock dimensioned geometry;
- approve a numeric load case;
- record a mechanism or material decision;
- add test evidence.

Topology-only expansion does not advance these gates.
