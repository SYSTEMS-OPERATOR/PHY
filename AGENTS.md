# AGENTS.md

PHY local-agent operating specification (off-grid, manufacturing-first).

## Mission
PHY provides a fabrication-oriented digital skeleton pipeline: canonical bone data, deterministic assembly, validation, and export for review and manufacturing planning.

## Layer Authority
- Fabrication-critical truth: `BODY.md`
- Runtime behavior + orchestration: `MIND.md`
- Symbolic/persona framing: `SOUL.md`

Rules:
- symbolic/metaphysical logic belongs in `SOUL.md` or optional `skeleton/extensions/soul/`
- fabrication truth belongs in `BODY.md`
- runtime orchestration belongs in `MIND.md`

## Directory Map (Expected)
- `BODY.md`, `MIND.md`, `SOUL.md`, `README.md`, `AGENTS.md`
- `assemble_skeleton.py`
- `skeleton/`
  - `schema/` (authoritative schema)
  - `bones/` (bone modules)
  - `datasets/` (local datasets)
  - `agents/` (legacy/simple agent utilities)
  - `validation/` (fabrication validator)
  - `exporters/` (deterministic exports)
  - `extensions/soul/` (optional symbolic overlays)
- `bin/` (CLI workflows)
- `dist/` (machine outputs)
- `exports/` (exchange outputs)
- `reports/` (validation and review outputs)
- `references/` (traceability sources)

## Authoritative vs Generated Files
Authoritative (human-edited):
- `BODY.md`, `MIND.md`, `SOUL.md`
- `skeleton/schema/bone.schema.json`
- `skeleton/base.py`, `skeleton/validation/validator_agent.py`, `skeleton/exporters/exporter_agent.py`
- per-bone modules under `skeleton/bones/`

Generated (safe to regenerate):
- `dist/*`
- `exports/*`
- `reports/validation_report.*`

Must not be auto-edited without explicit intent:
- schema contract (`skeleton/schema/bone.schema.json`)
- root authority docs (`BODY.md`, `MIND.md`, `SOUL.md`)

## Canonical Invariants
- One canonical bone model: `BoneSpec` in `skeleton/base.py`
- Required fields match `skeleton/schema/bone.schema.json`
- canonical units: mm, kg, kg/m^3, kg*m^2
- no fabrication validation depends on symbolic state
- deterministic exports from identical repo state

## Agent Roles
- **BoneAgent**: bone-level data wrapper (`BoneSpec` and `to_fabrication_record`)
- **AssemblerAgent**: hierarchy assembly (`assemble_skeleton.py`, field loading)
- **ValidatorAgent**: fabrication-grade checks + report generation
- **ReferenceAgent**: traceability + citation completeness
- **ExporterAgent**: JSON / URDF-like / TF / BOM / tables / audits
- **InspectorAgent**: human-readable markdown review packet in `reports/`
- **VisualizerAgent** (optional): simple visualization pipeline

## Validation Commands
- `PYTHONPATH=. bin/validate_fabrication.py`
- Outputs:
  - `reports/validation_report.json`
  - `reports/validation_report.md`

## Export Commands
- `PYTHONPATH=. bin/export_fabrication.py`
- Full pass: `PYTHONPATH=. python3 assemble_skeleton.py`
- Outputs include canonical skeleton JSON, TF tree, URDF-like structure, BOM, material table, joint table, reference audit.

## Safe Edit Boundaries
- Keep fabrication-critical logic isolated from symbolic language.
- Preserve backward compatibility where practical for existing bone modules.
- Never invent unknown measurements; report as missing.

## Off-grid / Local Model Behavior
- Assume no internet and no cloud inference.
- Use local repo files as sole source of fabrication truth.
- If model confidence is low, emit explicit uncertainty in reports and fail validation where appropriate.

