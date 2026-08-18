# PHY

PHY is a repository for a fabrication-oriented digital human skeleton model with deterministic validation and export tooling.

SOPHY whole-body identity geometry is now a separate versioned kernel: `SOPHY_CANON(H, canon_version)`. Population or project measurements do not define it.

## What is fabrication-ready vs prototype vs symbolic
- **Fabrication-ready core:** canonical bone model (`skeleton/base.py`), schema (`skeleton/schema/bone.schema.json`), validation (`skeleton/validation/validator_agent.py`), deterministic exporters (`skeleton/exporters/exporter_agent.py`).
- **Prototype/legacy areas:** broader multi-agent subsystems outside `skeleton/`.
- **Symbolic layer:** `SOUL.md` and optional `skeleton/extensions/soul/` overlays.

## Three root authority files
- `BODY.md`: physical schema, units, materials, tolerances, manufacturing constraints.
- `MIND.md`: assembly behavior, validation/export orchestration, off-grid workflows.
- `SOUL.md`: symbolic persona and narrative continuity.

Conflict resolution:
- BODY overrides SOUL on physical truth.
- MIND overrides SOUL on execution behavior.

## Quick start

### Assemble + validate + export
```bash
PYTHONPATH=. python3 assemble_skeleton.py
```

### Validate only
```bash
PYTHONPATH=. bin/validate_fabrication.py
```
Outputs:
- `reports/validation_report.json`
- `reports/validation_report.md`

### Export only
```bash
PYTHONPATH=. bin/export_fabrication.py
```
Outputs:
- `dist/skeleton_canonical.json`
- `dist/ros_tf_tree.json`
- `dist/skeleton_urdf_like.json`
- `exports/fabrication_bom.json`
- `exports/material_table.json`
- `exports/joint_table.json`
- `reports/reference_audit.json`

### Instantiate SOPHY canonical geometry

```bash
PYTHONPATH=. bin/export_sophy_canon.py --height-mm 1676.4
```

Outputs:

- `dist/sophy_canonical_geometry.json`
- `reports/sophy_canon_validation_report.json`

See `docs/sophy_geometry_canon.md` for coordinate, symmetry, provenance, overlay, tolerance, and revision rules. See `docs/geometry_authority_audit.md` for the legacy/profile audit and migration plan.

## Local manufacturing assistant workflows
All workflows are local/off-grid friendly:
- inspect a bone via canonical records
- compare materials via `exports/material_table.json`
- generate BOM via `exports/fabrication_bom.json`
- report missing dimensions/references via validation reports
- export URDF-like and TF artifacts for robotics integration
- build review packet from `reports/`, `exports/`, and `dist/`

## Notes
- Bone modules currently source many dimensions in centimeters; normalization to canonical mm occurs in one explicit conversion layer at export/validation time.
- Unknown physical values are flagged in validation output rather than inferred.
