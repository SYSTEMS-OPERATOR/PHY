#!/usr/bin/env python3
"""Export source-dimensioned review solids; never infer assembly geometry.

Run from any directory: python bin/export_component_3d.py [--output DIR]
Requires CadQuery 2.7.0. Viewer is self-contained and works offline.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BASELINE = "skeleton/datasets/female_21_baseline.json"
BUSHINGS = "PROJECTS/REDWOOD/hardware/bushing_catalog.json"
GUIDE = "PROJECTS/REDWOOD/bone_guides/BONE_FEMUR_R.md"
REGISTER = "PROJECTS/T56_CARBON/requirements/geometry_register.json"


def read(path):
    return json.loads((ROOT / path).read_text())


def positive(value):
    return type(value) in (int, float) and math.isfinite(value) and value > 0


def component_specs():
    """Bounding dimensions only: no invented curvature or anatomical placement."""
    specs, omitted = [], []
    for name, row in read(BASELINE).items():
        if not isinstance(row, dict) or name in ("BoneMaterialProperties", "SyntheticMaterials"):
            continue
        keys = [next((k for k in pair if k in row), pair[0]) for pair in (
            ("width_cm", "avg_width_cm"),
            ("thickness_cm", "avg_thickness_cm"),
            ("length_cm", "avg_height_cm", "avg_length_cm"),
        )]
        if not all(positive(row.get(k)) for k in keys):
            omitted.append({"id": name, "reason": "No unambiguous width/thickness/length triple; axis remapping not inferred."})
            continue
        specs.append({"id": "REF_" + name.upper(), "name": name + " reference envelope",
                      "kind": "box", "xyz_mm": [row[k] * 10 for k in keys],
                      "source": BASELINE, "source_keys": [name + "." + k for k in keys],
                      "authority": "legacy biological reference; not SOPHY canon or finished bone geometry",
                      "assembly_transform": None})
    # Extract the existing guide's blank, rather than manufacturing new allowances.
    import re
    guide = (ROOT / GUIDE).read_text()
    blank = {}
    for axis in ("length", "width", "thickness"):
        match = re.search(r"Blank " + axis + r": `([0-9.]+) mm`", guide)
        if not match or not positive(float(match[1])):
            raise ValueError("Missing positive femur blank dimension: " + axis)
        blank[axis] = float(match[1])
    specs.append({"id": "REDWOOD_FEMUR_BLANK", "name": "REDWOOD femur rough blank",
                  "kind": "box", "xyz_mm": [blank[k] for k in ("width", "thickness", "length")],
                  "source": GUIDE, "source_keys": ["Rough Redwood blank"],
                  "authority": "provisional REDWOOD blank; joint centers and drilling unresolved",
                  "assembly_transform": None})
    for row in read(BUSHINGS)["items"]:
        dims = [row[k] for k in ("inner_diameter_mm", "outer_diameter_mm", "nominal_length_mm")]
        if not all(positive(v) for v in dims) or dims[0] >= dims[1]:
            raise ValueError("Invalid bushing dimensions: " + row["part_id"])
        specs.append({"id": row["part_id"], "name": row["part_id"].replace("_", " "),
                      "kind": "bushing", "id_od_length_mm": dims,
                      "xyz_mm": [dims[1], dims[1], dims[2]], "source": BUSHINGS,
                      "source_keys": [row["part_id"]],
                      "authority": "nominal catalog geometry; fits, tolerances and placement unresolved",
                      "assembly_transform": None})
    return specs, omitted


def build_shape(spec):
    import cadquery as cq
    if spec["kind"] == "box":
        return cq.Workplane("XY").box(*spec["xyz_mm"], centered=(True, True, False)).val()
    inner, outer, length = spec["id_od_length_mm"]
    return cq.Workplane("XY").circle(outer / 2).circle(inner / 2).extrude(length).val()


def export(output):
    import cadquery as cq
    output.mkdir(parents=True, exist_ok=True)
    for folder in ("step", "stl"):
        (output / folder).mkdir(exist_ok=True)
    specs, omitted = component_specs()
    meshes, checks = [], []
    for spec in specs:
        shape = build_shape(spec)
        box = shape.BoundingBox()
        actual = [box.xlen, box.ylen, box.zlen]
        expected_volume = math.prod(spec["xyz_mm"])
        if spec["kind"] == "bushing":
            inner, outer, length = spec["id_od_length_mm"]
            expected_volume = math.pi * (outer**2 - inner**2) / 4 * length
        valid = (shape.isValid() and len(shape.Solids()) == 1
                 and all(abs(a-b) < 1e-6 for a, b in zip(actual, spec["xyz_mm"]))
                 and math.isclose(shape.Volume(), expected_volume, rel_tol=1e-9))
        if not valid:
            raise ValueError("Invalid solid: " + spec["id"])
        step = output / "step" / (spec["id"] + ".step")
        cq.exporters.export(shape, str(step))
        cq.exporters.export(shape, str(output / "stl" / (spec["id"] + ".stl")), tolerance=0.02, angularTolerance=0.1)
        reloaded = cq.importers.importStep(str(step)).val()
        if not reloaded.isValid() or not math.isclose(reloaded.Volume(), expected_volume, rel_tol=1e-7):
            raise ValueError("STEP round trip failed: " + spec["id"])
        vertices, faces = shape.tessellate(0.05, 0.1)
        meshes.append({**spec, "vertices": [list(v.toTuple()) for v in vertices], "faces": faces})
        checks.append({"id": spec["id"], "valid_solid": True, "bounds_mm": actual,
                       "volume_mm3": shape.Volume(), "step_round_trip": "pass"})
    register = read(REGISTER)
    manifest = {"artifact": "PHY_SOURCE_COMPONENT_REVIEW_KIT", "units": "mm",
                "fabrication_ready": False, "assembled_body": False,
                "representation": "Unplaced local solids. Boxes are dimension envelopes, not anatomical meshes.",
                "local_frame": "X=source width, Y=source thickness, Z=source length; not anatomical axes. Bottom center origin is a display convention.",
                "no_rescaling": "REDWOOD and biological dimensions retained as recorded; not rescaled to T56.",
                "canon": read("PROJECTS/T56_CARBON/profiles/t56_domestic_frame.json")["canon_reference"],
                "components": specs, "omitted": omitted,
                "open_t56_inputs": [r for r in register["parameters"] if r["value"] is None],
                "source_sha256": {p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in (BASELINE, BUSHINGS, GUIDE, REGISTER)},
                "geometry_checks": checks}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    template = (ROOT / "skeleton/visualization/component_viewer.html").read_text()
    payload = json.dumps(meshes).replace("<", "\\u003c")
    (output / "viewer.html").write_text(template.replace("__COMPONENT_DATA__", payload))
    print(f"Exported {len(specs)} valid source-dimensioned solids to {output}; not an assembled or fabrication-released body.")
    return manifest


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "exports/component_3d")
    export(parser.parse_args().output)
