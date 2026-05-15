# DEFAULT Project Overlay

DEFAULT is a PHY-compatible project overlay for prototype armature fabrication using ordinary, easily sourced shop materials before final species-specific cuts.

## Design rule

Core PHY stays universal. DEFAULT stays project-specific.

Do not hard-code DEFAULT assumptions into:

- `skeleton/base.py`
- `skeleton/schema/bone.schema.json`
- `skeleton/bones/`
- `BODY.md`
- `MIND.md`

Project-specific truth belongs here under `PROJECTS/DEFAULT/`.

## MVP purpose

The MVP should let a builder inspect each component and produce draft blanks, drill tests, hardware tests, and validation passes before committing to final material.

Candidate materials may include dimensional softwood, Douglas-fir, poplar, maple, birch plywood, Baltic birch plywood, and other common prototype stock.

## Required outputs

- `reports/DEFAULT_BUILD_BOOK.md`
- `reports/DEFAULT_VALIDATION.md`
- `cutlists/default_cut_list.csv`
- `cutlists/drill_schedule.csv`
- `cutlists/hardware_cut_list.csv`
- `simulation/default_armature_urdf_like.json`
- one component guide per buildable component or an explicit missing-data record
