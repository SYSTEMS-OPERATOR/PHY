#!/usr/bin/env python3
"""Write a reproducible A0 single-side convergence snapshot."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from PROJECTS.T56_CARBON.tools.check_convergence import audit


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    result = audit("single-side")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "A0_CONVERGENCE.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    counts = result["counts"]
    lines = [
        "# T56 A0 convergence snapshot", "",
        f"Schema valid: **{str(result['schema_valid']).lower()}**. Article ready: **{str(result['article_ready']).lower()}**.", "",
        "| Gate | Closed | Total |", "| --- | ---: | ---: |",
        f"| Mission inputs | {counts['mission_inputs_in_scope'] - counts['open_mission_inputs_in_scope']} | {counts['mission_inputs_in_scope']} |",
        f"| Geometry inputs | {counts['geometry_parameters_in_scope'] - counts['open_geometry_parameters_in_scope']} | {counts['geometry_parameters_in_scope']} |",
        f"| Approved load cases | {counts['approved_load_cases_in_scope']} | {counts['load_cases_in_scope']} |",
        f"| Approved evidence packages | {counts['approved_article_evidence_packages']} | {counts['article_evidence_packages_in_scope']} |",
        "", "The six load cases have closed-form results and numeric acceptance criteria, but remain unapproved until signed physical records exist. Evidence packages list design artifacts and deliberately blank templates; none is approved.",
        "", "## Remaining blockers", "",
    ] + [f"- {item}" for item in result["blockers"]]
    (args.output_dir / "A0_CONVERGENCE.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("schema_valid" if result["schema_valid"] else "schema_invalid")
    return 0 if result["schema_valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
