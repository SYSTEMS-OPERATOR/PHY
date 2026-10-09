# Canonical export packages

`ExporterAgent` publishes a deterministic package from a copied `SkeletonField`.
It converts every record, runs the canonical record gate and resolves exported
identities before writing. Unknown measurements remain JSON `null`. Nonfinite
numbers, failed conversions and invalid or duplicate bone IDs prevent publication
in every mode; an unsuccessful attempt never returns success paths or a subset.

## Modes and entrypoints

Both `bin/export_fabrication.py` and `assemble_skeleton.py` use the same CLI:

```bash
# Default: canonical record and identity gates must both pass.
PYTHONPATH=. python bin/export_fabrication.py --output-root /tmp/phy-validated

# Explicitly publish an incomplete model for review.
PYTHONPATH=. python assemble_skeleton.py --mode review --output-root /tmp/phy-review

# A different local dataset, with the same gates.
PYTHONPATH=. python bin/export_fabrication.py --dataset female_21_baseline --mode review
```

| Mode | Canonical record gaps | Unresolved anatomical references or parent cycles | Invalid IDs, conversion errors or nonfinite JSON |
| --- | --- | --- | --- |
| `review` | Published with blockers | Published with diagnostics | Publication blocked |
| `validated` (CLI default) | Publication blocked | Publication blocked | Publication blocked |

The current 193-record model remains blocked in validated mode. This export
change supplies no missing osteometry, mount definitions or joint evidence.
Review mode writes the same record and identity reports as validated mode.
`validated_ready` records whether both gates pass, regardless of the chosen mode.

Exit 0 and `published: true` mean the package was published. Exit 1 and
`published: false` mean the attempt failed. The CLI writes the attempt diagnostic
to `reports/export_failure.json`, outside the manifest, and reports its path in
stdout. If that diagnostic cannot be written, stdout includes `diagnostic_error`
and a null diagnostic path. Prior package files are preserved on input, validation
and serialization failure. An old failure diagnostic may remain after a later
success; package status comes from the verified manifest and its named reports.

The Python API retains its positional directory arguments and **review default**
for existing callers. Select the gate explicitly when required:

```python
from pathlib import Path
from skeleton.bones import load_field
from skeleton.exporters import ExportError, ExporterAgent

try:
    paths = ExporterAgent(load_field()).export_all(
        Path("dist"), Path("reports"), Path("exports"), mode="validated",
    )
except ExportError as error:
    # JSON-safe attempt diagnostics; no successful package paths.
    print(error.code, error.report)
```

## Stable identity and anatomical references

All machine identities use `BoneSpec.unique_id`: canonical records, frame IDs,
link names, BOM part numbers, material/joint table bone columns and reference
audit entries. Human-readable names remain labels and may repeat for bilateral
bones. Export joint IDs are `<bone_id>__joint_<zero-based interface index>`;
interface names remain display labels. Reordering interfaces changes those IDs.

Reference resolution is exact and deterministic:

1. An existing unique bone ID wins.
2. An exact legacy display name resolves only when it identifies one bone.
3. Missing, malformed or ambiguous references remain unresolved, with candidates
   reported for ambiguities. The exporter does not infer side, spelling or case.

Canonical exported `connections` contain only resolved IDs and have role
`anatomical_references`. `source_connections` retain the original declarations;
`connection_resolution` retains each reference and its status. Identity diagnostics
are in `export_report.json`, including self-parent and longer parent cycles.

TF-like parent/child relationships come only from declared `connections.parent`.
Declared `connections.children` are anatomical adjacency references; they are
resolved and audited separately, rather than interpreted as additional parents.
Multiple roots and missing parent declarations do not establish a mechanical
assembly. Anatomical adjacency can include loops.

The TF-like inventory always has null `origin_mm` and `rotation_rpy_rad`, with
`transform_status: unresolved`. A source primitive's geometry origin is not
evidence of an assembly pose. The URDF-like inventory marks joint definitions
`unqualified`, preserves a supplied type and leaves an absent type null. It does
not assign a fixed joint to a floating scapula or manufacture zero transforms.
`mechanical_transforms_validated` remains false.

These are intentional compatibility changes: machine columns formerly containing
display names now contain IDs, missing poses no longer appear as zeros, and an
absent joint type no longer appears as `fixed`. The files are JSON inventories,
not runnable ROS transforms or a mechanically complete URDF.

## Files and traceability

A successful export returns ten paths. The seven original artifacts remain,
with three additions: the canonical validation report, export report and manifest.

| Path | Contents |
| --- | --- |
| `dist/skeleton_canonical.json` | Canonical records, source references and resolved identities |
| `dist/ros_tf_tree.json` | Declared anatomical parent inventory with unresolved transforms |
| `dist/skeleton_urdf_like.json` | Links and unqualified interface definitions |
| `exports/fabrication_bom.json` | Stable part IDs, material, revision and tolerance |
| `exports/material_table.json` | Bone IDs, labels and material properties |
| `exports/joint_table.json` | Bone and interface IDs with source joint labels |
| `reports/reference_audit.json` | Missing source references by bone ID |
| `reports/validation_report.json` | Canonical record gate results |
| `reports/export_report.json` | Chosen mode, record blockers and identity gate results |
| `dist/export_manifest.json` | Package contract version, readiness and content hashes |

The manifest's `files` entries use paths relative to the manifest directory and
SHA-256 hashes of the exact nine artifact byte streams. It excludes itself and
the separate failure diagnostic. `source` contains a fingerprint of the original
canonical records before export identity resolution, the authoritative schema's
byte hash, bone revisions and deduplicated fingerprints of attached dataset
contents grouped by bone ID. These identify the data actually consumed; they do
not claim a dataset filename, Git commit, measurement citation or reviewed release.

All JSON uses sorted keys, strict numbers, two-space indentation, UTF-8 and a
final newline. Record and report order is deterministic by bone ID. Identical
inputs and output directory layout produce identical package bytes.

## Publication and release limits

Every payload is serialized before any destination is changed. Files are staged
in their destination directories, previous files are backed up, and the manifest
is replaced last. Ordinary I/O failure triggers rollback and removal of newly
published files. A rollback failure reports `previous_package_restored: false`,
its errors and retained backup paths for recovery; that package is not accepted.

Publication requires exclusive access to the destination. This is not an atomic
transaction across directories: process termination or power loss can interrupt
replacement. Consumers must verify every manifest hash before accepting a package,
and ensure no publisher changes files while they read them. Hash mismatch means
the package is incomplete or mixed, regardless of its readiness field.

The validation scope is `canonical_records_and_export_identity`. Even a passing
synthetic fixture exports `fabrication_released: false`. The gate does not verify
mechanical transforms, articular/contact topology, measured anatomy, qualified
interfaces, inertia, strength, swept clearance or article release evidence.

Run the focused software regressions using the standard library:

```bash
PYTHONPATH=. python -m unittest discover -s tests -p 'test_fabrication_*.py' -v
```

Both Python-package CI (3.11) and Studio CI (3.12) include these regressions.
