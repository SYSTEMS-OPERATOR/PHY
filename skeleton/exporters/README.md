# 📦 The loading dock

*Where PHY's bones get their paperwork.*

Need a JSON skeleton, a bill of materials, or SOPHY's mathematical identity?
Pick a door. Nulls travel first class here.

| Door | What happens inside |
| --- | --- |
| [__init__.py](__init__.py) | Public imports: `ExporterAgent`, `ExportError` and `CanonicalGeometryExporter`. |
| [exporter_agent.py](exporter_agent.py) | Builds the twelve-file bone-record package, including typed relationships and their inspection report. |
| [relationship_exporter.py](relationship_exporter.py) | Publishes a standalone relationship graph/report with a hash manifest. |
| [identity.py](identity.py) | Checks the guest list: exact bone IDs first, unique exact display names second; reports ambiguous references and parent cycles. |
| [publication.py](publication.py) | Stages record-package files, publishes the manifest last and attempts rollback after ordinary I/O failures. |
| [cli.py](cli.py) | Shared entrypoint behind [export_fabrication.py](../../bin/export_fabrication.py) and [assemble_skeleton.py](../../assemble_skeleton.py). |
| [canon_exporter.py](canon_exporter.py) | Validates and writes the separate, renderer-neutral SOPHY canon artifact. |

## Choose a shipping lane

Run this from the repository root to inspect the current bone records:

```sh
PYTHONPATH=. python bin/export_fabrication.py --mode review --output-root /tmp/phy-review
```

- **`review`** preserves known gaps and unresolved references in an inspectable
  package. Invalid identities or nonfinite JSON still block publication.
- **`validated`** requires both record and export-identity gates. This is the
  CLI default; the Python `ExporterAgent.export_all()` API defaults to `review`.

Every successful record package keeps `fabrication_released: false`.
Its TF poses remain unresolved, and its URDF-like inventory carries unqualified
joint definitions. A green export is a statement about its declared validation
scope; physical release requires its own evidence and approval.

## Before the packing tape

Verify the manifest's artifact hashes. Give a destination package one writer
at a time: publishing across several directories is not a single atomic rename,
and process/power interruption can leave an incomplete publication.

For the identity geometry, start at [the canon CLI](../../bin/export_sophy_canon.py).
SOPHY canon **1.0.0** is deterministic and mirror-derived. Scale, materials and
mechanisms may change; canon does not drift.

## Further along the conveyor belt

- [Export contract and publication limits](../../docs/fabrication_exports.md)
- [Fabrication record validator](../validation/validator_agent.py)
- [SOPHY geometric canon](../../docs/sophy_geometry_canon.md)
- [Record-package regression checks](../../tests/test_fabrication_exports.py)
- [Canon regression checks](../../tests/test_sophy_geometry_canon.py)

*Paperwork before packing tape. — SOPHY 📦*
