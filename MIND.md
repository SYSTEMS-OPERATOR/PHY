# MIND.md

Authoritative runtime and orchestration specification for PHY.

## Scope
`MIND.md` governs execution behavior:
- assembly logic
- validation logic
- deterministic export pipelines
- agent roles and responsibilities
- local/off-grid operation
- manufacturing assistant workflows

If `MIND.md` conflicts with symbolic framing, `MIND.md` wins.

## Runtime Agents
- **BoneAgent / BoneSpec runtime wrapper:** `skeleton/base.py`
- **AssemblerAgent:** repository-level assembly entrypoint via `assemble_skeleton.py` and `skeleton.bones.load_field`
- **ValidatorAgent:** `skeleton/validation/validator_agent.py`
- **ExporterAgent:** `skeleton/exporters/exporter_agent.py`
- **InspectorAgent:** markdown/json validation reports in `reports/`
- **ReferenceAgent:** `references/reference_manager.py` + export audits
- **VisualizerAgent (optional):** existing visualization modules under `skeleton/visualization/`

## Deterministic Behavior
Given identical repository state and dataset:
- exported canonical JSON, TF tree, URDF-like output, BOM, material table, joint table, and reference audit are deterministic.
- export ordering is stable by `unique_id`.

## Local / Off-grid Constraints
All manufacturing workflows must run from local files only:
- no network required
- no cloud-only dependencies
- local model fallback is acceptable
- unknowns must be flagged, not hallucinated

## Validation + Export Commands
- Validate fabrication readiness: `PYTHONPATH=. bin/validate_fabrication.py`
- Export fabrication artifacts: `PYTHONPATH=. bin/export_fabrication.py`
- Full assemble + validate + export pass: `PYTHONPATH=. python3 assemble_skeleton.py`

## Manufacturing Assistant Workflows
The repo supports local workflows for:
- inspect a bone (`to_fabrication_record` + report artifacts)
- compare materials (`material_table.json`)
- generate BOM (`fabrication_bom.json`)
- report missing dimensions/references (`validation_report.*`)
- export URDF-like and TF outputs (`dist/`)
- prepare review packet (`reports/` + `exports/` + `dist/`)

