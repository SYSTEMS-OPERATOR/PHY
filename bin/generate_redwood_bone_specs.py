#!/usr/bin/env python3

import json
from pathlib import Path

# Get repo root (bin/ is one below repo root)
REPO_ROOT = Path(__file__).resolve().parent.parent
PROJ_PROFILE_PATH = REPO_ROOT / "PROJECTS/REDWOOD/profiles/adult_female_21_28.json"
BIO_BASELINE_PATH = REPO_ROOT / "skeleton/datasets/female_21_baseline.json"
GUIDE_OUT_DIR = REPO_ROOT / "PROJECTS/REDWOOD/bone_guides"
GUIDE_OUT_DIR.mkdir(exist_ok=True, parents=True)

def load_json(path):
    with open(path, "r") as f:
        return json.load(f)

def mm(val):
    if isinstance(val, (float, int)):
        return int(round(val))
    return "N/A"

def get_bone_side_pairs(anthro_data):
    pairs = []
    # Long bones (have L & R)
    for bone in ["Femur", "Tibia", "Fibula", "Humerus", "Radius", "Ulna"]:
        for side in ("L", "R"):
            pairs.append((bone, side))
    # Single bones (midline)
    for bone in ["Clavicle", "Scapula", "HipBone", "Vertebra", "Sacrum"]:
        pairs.append((bone, None))
    return pairs

def get_project_envelope(profile, bone_name):
    key = bone_name.lower()
    plen = profile.get("primary_lengths_mm", {}).get(key, None)
    pwidth = None
    pthick = None
    # fallback for width/thickness
    if bone_name == "Femur":
        pwidth = plen and 40 or None
        pthick = plen and 40 or None
    return plen, pwidth, pthick

def get_side_prefix(side):
    return f"_{side}" if side else ""

BONE_CLASS = {
    "Femur": "long_bone",
    "Tibia": "long_bone",
    "Fibula": "long_bone",
    "Humerus": "long_bone",
    "Radius": "long_bone",
    "Ulna": "long_bone",
    "Clavicle": "flat_bone",
    "Scapula": "flat_bone",
    "HipBone": "irregular_bone",
    "Vertebra": "irregular_bone",
    "Sacrum": "irregular_bone",
}

def fabricate_guide(bone, side, proj, bio):
    side_s = f"_{side}" if side else ""
    unique_id = f"BONE_{bone.upper()}{side_s}"
    bone_label = bone
    side_human = side.lower() if side else "midline"
    plen, pwidth, pthick = get_project_envelope(proj, bone)
    bio_data = bio.get(bone, {})
    bio_len = bio_data.get("length_cm", bio_data.get("avg_height_cm", None))
    bio_width = bio_data.get("width_cm", bio_data.get("avg_width_cm", None))
    bio_thick = bio_data.get("thickness_cm", bio_data.get("avg_thickness_cm", None))
    bio_mass = bio_data.get("mass_g", bio_data.get("avg_mass_g", None))
    bio_density = bio_data.get("density_kg_m3", None)
    blank_len = mm(plen) + 10 if plen else ""
    blank_width = mm(pwidth) + 4 if pwidth else ""
    blank_thick = mm(pthick) + 4 if pthick else ""
    md = f"""# {unique_id} — {bone_label} ({side_human})

## Purpose

Fabrication guide for {side_human} {bone_label} as a REDWOOD reference.  
Auto-generated from project anthropometric and anatomical datasets.

## Finished target envelope

- Finished length: `{mm(plen)} mm` *(see: adult_female_21_28.json)*
- Finished width: `{mm(pwidth)} mm`
- Finished thickness: `{mm(pthick)} mm`
- Target side: `{side_human}`
- Bone class: `{BONE_CLASS.get(bone, '')}`

## Biological reference (21-year-old female, avg)

- Biological length: `{bio_len} cm`
- Biological width: `{bio_width} cm`
- Biological thickness: `{bio_thick} cm`
- Mass: `{bio_mass} g`
- Density: `{bio_density} kg/m³`

## Rough Redwood blank

- Blank length: `{blank_len} mm`
- Blank width: `{blank_width} mm`
- Blank thickness: `{blank_thick} mm`
- Grain axis: `proximal_to_distal`
- Allowance notes: `10 mm length, 4 mm width/thickness added`

## Carving envelope

- Preserve joint regions until final drilling.
- Maintain minimum dimension at joint ends.
- Suggested centerline: follow grain.

## Joint centers

| End      | Joint center                | Hardware group | Drill target           | Notes                          |
|----------|-----------------------------|---------------|------------------------|---------------------------------|
| Proximal | TBD (see parent module)     | large_primary | 12 mm bushing OD      | To be set by joint module      |
| Distal   | TBD (see child module)      | large_primary | 12 mm bushing OD      | Align with adjacent bone guide |

## Hardware

- Pins: `COPPER_PIN_8MM`
- Bushings: `BRASS_BUSHING_8MM_ID_12MM_OD`
- Washers/spacers: `BRASS_WASHER_8MM`
- Hinge/plate: `BRASS_SINGLE_AXIS_HINGE_LARGE` (if articulated)

## Drill schedule

| Operation                  | Diameter     | Depth    | Reference face        | Notes                           |
|----------------------------|-------------|----------|----------------------|----------------------------------|
| {unique_id}_HIP_BUSHING_PILOT   | 12 mm      | TBD      | proximal end         | Validate with parent module      |
| {unique_id}_KNEE_BUSHING_PILOT  | 12 mm      | TBD      | distal end           | Validate with distal module      |

## Simulation fields

- Approximate mass (wood): see project calculation
- Center of mass: ~45% length from proximal end (long bones)
- Joint axis: see project joint module
- Joint limits: as per joint module

## Source datasets

- Build values: [`profiles/adult_female_21_28.json`](../profiles/adult_female_21_28.json)
- Biological values: [`skeleton/datasets/female_21_baseline.json`](../../../skeleton/datasets/female_21_baseline.json)

---
"
    md_name = f"BONE_{bone.upper()}{side_s}.md"
    with open(GUIDE_OUT_DIR / md_name, "w") as out:
        out.write(md)

def main():
    proj = load_json(PROJ_PROFILE_PATH)
    bio = load_json(BIO_BASELINE_PATH)
    for bone, side in get_bone_side_pairs(bio):
        fabricate_guide(bone, side, proj, bio)
    print(f"Bone fabrication spec files generated in {GUIDE_OUT_DIR}")

if __name__ == "__main__":
    main()
