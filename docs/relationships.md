# PHY-REL-001 / typed relationship inspection

Relationship contract `1.0.0` observes a supplied BoneSpec/runtime snapshot and
the existing craniofacial catalog. It supplies no geometry, pose, joint axis,
mechanical limit, qualification or fabrication release. SOPHY canon `1.0.0`
remains unchanged: H = span = 1676.4 mm, deterministic and mirror-derived.
Scale, materials and mechanisms may change; canon does not drift.

## Separate meanings

| Kind | Authority and interpretation |
| --- | --- |
| `legacy_parent` | Canonical `connections.parent`; a declared anatomical reference, not a mechanical parent. Only this projection participates in parent-cycle checking. Null means unspecified. |
| `legacy_adjacency` | Canonical `connections.children`; adjacency declarations can form ordinary anatomical loops. |
| `legacy_articulation` | Original `articulations` entries, including the legacy joint label. No automatic biological or engineering interpretation is added. |
| `qualitative_articulation` | The two catalog TMJs, with biological classification, qualitative rotation/translation, disc participation and null engineering definitions kept separate. Both share one adult mandible. |
| `structural_membership` | Seven catalog regions with participant roles. These are hyperedges for regional participation, never generated pairwise contacts, sutures or joints. Regions are not bones. |
| `operational_signal` | The supplied snapshot's directed runtime link entries. Runtime domain aliases resolve back to canonical bone IDs. Observing these links does not instruct routing or turn anatomical adjacency into routing. |

The legacy mandible's `temporal`/`hinge` entry survives beside the curated TMJs.
Exact matching reports that lower-case target missing; runtime compatibility
matching reports the two temporal candidates as ambiguous. Neither result
selects a side or upgrades the hinge label to an engineering definition.
The hyoid's unspecified parent stays unspecified.

## Python APIs and matching

`skeleton.identity.IdentityResolver.resolve(reference, policy=...)` implements
exact-ID-first resolution. Policies are `exact`, `case_insensitive` (the existing
runtime `lower()` rule), and `id_only`. Invalid and duplicate IDs are explicit
diagnostics; non-string references are never coerced. Runtime wiring uses the
case-insensitive policy. Rendering and ExportIdentity use exact matching.
The parent projection retains its independent cycle checks.

`skeleton.relationships.build_relationship_snapshot(field)` does not call a
wiring method, evaluate motion, mutate source objects or alter runtime state.
`relationships_for_bone(graph, bone_id)` includes resolved participants and
unresolved declarations naming the ID as an ambiguity candidate.

The graph always accounts for the 206 adult inventory slots. Its coverage
counts only supplied adult records; missing records remain missing. Supplied
non-inventory records appear separately and do not enlarge the denominator.
Catalog participants missing from a partial snapshot remain unresolved.

Declaration IDs use bone ID plus original field/index, or catalog entity ID.
Record ordering is normalized by ID; source list order remains part of a
declaration's identity and runtime snapshot. Malformed JSON values survive;
non-JSON/nonfinite values receive explicit JSON-safe tags. No value is invented.

## Evidence and packages

Each declaration retains its original value, participant resolutions, semantic
role, source field/catalog locator, relevant source IDs and content hashes.
Fresh inputs include inventory, raw relationship/runtime fields, current source
module bytes, catalog and artifact schema. Source-module correspondence alone
does not prove that a caller-supplied object came from that file. Supplied
catalog objects are identified as supplied, without a false repository-file hash.
The catalog's existing bounded validation is shared with the craniofacial audit;
its publisher citations are declarations, not newly fetched evidence.

```
python bin/export_relationship_graph.py --output-root /tmp/phy-relationships
```

This publishes `relationship_graph.json`, `RELATIONSHIPS.md` and
`relationship_manifest.json` using the existing staging/rollback utility, with
the manifest last. There are no timestamps or absolute checkout/output paths in
artifact bytes. Hashes identify the actual JSON/Markdown bytes. Failures do not
constitute acceptance; verify the manifest before using a package.

The normal bone-record package is additive contract `1.1.0`: twelve files,
including `dist/relationship_graph.json` and `reports/relationship_graph.md`.
Its existing review/validated gates are unchanged. Partial relationship coverage
does not silently redefine those gates, and neither gate releases fabrication.

Studio embeds the same graph in engineering evidence, ships separate JSON and
Markdown inspection files, and hashes their actual bytes in its package manifest.
Its Engineering/component inspectors separate the kinds above. Participant
navigation selects an existing proxy; it changes no model geometry, scale, pose,
canon or mechanical limits. Review JSON retains the relationship graph alongside
the evidence. Display proxies remain unqualified anatomical geometry.
