# Craniofacial armature review: validation evidence

Date: 2026-10-12. Base: PR #107 merge
`da666467d30cf8bb25b66ac38951369bc5a02ac8`.
Branch: `agent/phy-craniofacial-relations`.

## Mission and verified result

Make the armature's structural relationship to external features inspectable
without adopting cosmetic ratios or unsupported geometry. The new offline
catalog covers seven selected regions and two TMJ sites sharing one adult
mandible. Seventeen existing canonical identities and their Studio proxies
participate. All seven spatial definitions and both mechanical definitions
remain null; external feature fit and facial canon remain unresolved/unbound.

The qualitative catalog is separate from BoneSpec connections and the selected
display adjacency hints. Orbital/palatal membership is not pairwise contact,
a parent transform, a joint interface or a complete suture graph. The mandible's
legacy ambiguous `temporal` references remain visible and unresolved.

The report separately labels the existing age-28 cohort's external head means
and the existing project head-height choice. It exposes their current stature
scaling as display design. Perception research is scoped to adjacent context;
the checker rejects its use for anatomical claims and adopts no numeric ratios.
Original citations/datasets behind the 2025 facial study discussions were not
recovered. The newly found perception paper was reviewed through its indexed
abstract only.

## Checks

| Check | Verified result |
| --- | --- |
| Full Python 3.12 suite | **278 passed, 5 skipped, 969 subtests passed** |
| New standard-library regressions | **11 tests passed**, without pytest/CadQuery |
| Catalog integrity | Required source scopes, skull IDs, sides, distinct region/joint IDs and null definition boundaries checked |
| TMJ semantics | One shared adult mandible; rotation and translation retained; disc reference and unresolved mechanism explicit |
| Snapshot preservation | Canonical records, source files, complete display model and input catalog preserved by audit |
| Independent CLI report repeats | All **three files byte-identical** across separate output roots |
| Tracked regional review | Matches both fresh repeats; manifest input and artifact hashes verified |
| CLI outside checkout | Passed; invalid height rejects and preserves prior package |
| Existing fabrication validator | Expected exit 1; **206 records, nine blocker categories**, readiness false |
| Existing export CLI | Default rejects; explicit review succeeds; subsequent rejection preserves prior package |
| Review transforms | All exported origins/rotations remain null; no registry transform enters the package |
| Viewer syntax | `npm run check --prefix studio` passed |
| Python syntax and whitespace | Passed |

The full suite includes existing source-fingerprint, skull/hand/foot/ear geometry,
bone equivalence, canonical cache, export-gate, SOPHY canon and A0 regressions.
No existing source measurements, source modules, geometry profiles, Studio
meshes/bundle/HTML, canonical schema/kernel, root authority documents, maquettes
or A0 article files are edited. Existing report/cache artifacts retain their
original hashes; only the new regional review package is generated here.

Four optional PyBullet modules and one Docker check were skipped. Python 3.11
and the browser are not available locally. Existing GitHub workflows retain
their Python 3.11, viewer build, GLB/STL, deterministic export and browser checks;
the Studio focused suite now includes the new craniofacial regressions.

## Reproduce

```bash
python -m pytest -q
python -m unittest discover -s tests -p test_craniofacial_audit.py -q
python bin/audit_craniofacial.py --output-root /tmp/phy-head-one
python bin/audit_craniofacial.py --output-root /tmp/phy-head-two
diff -qr /tmp/phy-head-one /tmp/phy-head-two
npm run check --prefix studio
```

The catalog/report command requires the Python standard library and local repo
data only. Geometry acquisition, actual module envelopes, soft-tissue offsets,
jaw trajectory, mechanical interfaces and physical acceptance remain future
evidence. Scope and the proposed next orbital/mandibular article are documented
in `docs/craniofacial_armature.md`. Prepared for review; no automatic merge or
branch deletion.
