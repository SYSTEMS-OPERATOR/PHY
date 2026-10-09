# PHY

PHY is a repository for a fabrication-oriented digital human skeleton model with deterministic validation and export tooling.

SOPHY whole-body identity geometry is now a separate versioned kernel: `SOPHY_CANON(H, canon_version)`. Population or project measurements do not define it.

## What is fabrication-ready vs prototype vs symbolic
- **Fabrication-oriented core:** canonical bone model (`skeleton/base.py`), schema (`skeleton/schema/bone.schema.json`), validation (`skeleton/validation/validator_agent.py`), deterministic exporters (`skeleton/exporters/exporter_agent.py`). Record validation and physical release are separate gates.
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

### Open PHY Studio

```bash
python bin/phy_studio.py
```

Explore a complete adult female reference armature, measured age-28 means and
slight form refinements, the SOPHY scale overlay, and the existing A0-R1 shoulder
CAD. Orbit, inspect parts, compare poses, measure and export 3D models. The portable
`studio/PHY-Studio.html` works offline and embeds a 1:4 maquette cut/assembly ZIP.
See `studio/README.md` and `PROJECTS/PHY_F28/README.md` for construction scope,
source provenance, rebuild commands and the remaining physical release gates.

### Assemble + validate + export
```bash
PYTHONPATH=. python3 assemble_skeleton.py
```

Assembly and export default to `--mode validated`. The current canonical model
has unresolved measurements and interfaces, so this exits 1 with
`reports/export_failure.json` and preserves any previous package. To publish an
explicit review package while retaining those gaps:

```bash
PYTHONPATH=. python3 assemble_skeleton.py --mode review
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
# Explicit review of incomplete records:
PYTHONPATH=. bin/export_fabrication.py --mode review
```
Successful package outputs:
- `dist/skeleton_canonical.json`
- `dist/ros_tf_tree.json`
- `dist/skeleton_urdf_like.json`
- `dist/export_manifest.json`
- `exports/fabrication_bom.json`
- `exports/material_table.json`
- `exports/joint_table.json`
- `reports/reference_audit.json`
- `reports/validation_report.json`
- `reports/export_report.json`

Both entrypoints accept `--dataset NAME` and `--output-root PATH`. The manifest
records the chosen mode, readiness, input fingerprints and artifact hashes.
Machine references use stable bone IDs; display names may repeat. TF-like and
URDF-like files are review inventories with unresolved mechanical transforms.
Every package declares `fabrication_released: false`. See
[`docs/fabrication_exports.md`](docs/fabrication_exports.md) for the contract,
compatibility changes and publication limits.

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
- inspect bone identities and declared anatomical connections in URDF-like and TF-like inventories
- build review packet from `reports/`, `exports/`, and `dist/`

## Notes
- Bone modules currently source many dimensions in centimeters; normalization to canonical mm occurs in one explicit conversion layer at export/validation time.
- Unknown physical values are flagged in validation output rather than inferred.
