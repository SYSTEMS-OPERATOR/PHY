# REDWOOD Project Overlay

REDWOOD is a PHY-compatible project overlay for a buildable redwood, copper, and brass adult female armature.

This directory must not redefine the universal PHY framework. It supplies one concrete fabrication target using PHY-compatible records, project-local materials, hardware, proportion profiles, bone guides, and offline build artifacts.

## Design rule

Core PHY stays universal. REDWOOD stays project-specific.

Do not hard-code redwood assumptions into:

- `skeleton/base.py`
- `skeleton/schema/bone.schema.json`
- `skeleton/bones/`
- `BODY.md`
- `MIND.md`

Project-specific truth belongs here under `PROJECTS/REDWOOD/`.

## MVP purpose

The MVP should let a builder inspect each bone and understand the base sizing required to rough-cut, drill, and carve a redwood armature part while preserving anatomical proportion and PHY simulation compatibility.

The builder may refine curves by eye, but the project must provide:

- finished target length, width, and thickness envelopes
- rough blank dimensions with fabrication allowance
- grain direction
- proximal and distal joint centers
- hardware assignments
- drill schedule
- adjacent bone connections
- simulation mass properties when enough data exists
- explicit unknowns where data is missing

## Target

- adult female armature
- age reference: 21-28
- neutral anatomical T-pose
- redwood primary structure
- copper/brass pins, collars, bushings, hinge plates, washers, and spacers
- offline fabrication and simulation use

## Definition of done

REDWOOD is considered MVP-ready when the project can generate or store:

- `reports/REDWOOD_BUILD_BOOK.md`
- `reports/REDWOOD_VALIDATION.md`
- `cutlists/redwood_cut_list.csv`
- `cutlists/drill_schedule.csv`
- `cutlists/hardware_cut_list.csv`
- `simulation/redwood_armature_urdf_like.json`
- one bone guide per buildable bone or an explicit missing-data record

Validation must fail if a part cannot be sized, cut, drilled, assembled, or simulated from local project data.
