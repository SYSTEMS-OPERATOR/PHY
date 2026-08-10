#!/usr/bin/env python3
"""Audit T-5.6 mission, geometry, and load-case convergence registers."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any


PROJECT_DIR = Path(__file__).resolve().parents[1]
REQUIREMENTS_DIR = PROJECT_DIR / "requirements"
REGISTER_PATHS = {
    "mission": REQUIREMENTS_DIR / "mission_envelope.json",
    "geometry": REQUIREMENTS_DIR / "geometry_register.json",
    "loads": REQUIREMENTS_DIR / "load_case_register.json",
}


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        value = json.load(handle)
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def nonempty(value: Any) -> bool:
    return value is not None and value != "" and value != [] and value != {}


def collect_unique_ids(groups: list[tuple[str, list[dict[str, Any]]]], errors: list[str]) -> set[str]:
    known: set[str] = set()
    for group_name, records in groups:
        for index, record in enumerate(records):
            record_id = record.get("id")
            if not isinstance(record_id, str) or not record_id:
                errors.append(f"{group_name}[{index}] has no valid id")
                continue
            if record_id in known:
                errors.append(f"duplicate id: {record_id}")
            known.add(record_id)
    return known


def require_locked(
    record: dict[str, Any],
    label: str,
    blockers: list[str],
    required_trace_fields: tuple[str, ...] = (),
) -> None:
    if record.get("status") != "locked" or not nonempty(record.get("value")):
        blockers.append(f"{label} is not locked with a value")
        return
    for field_name in required_trace_fields:
        if not nonempty(record.get(field_name)):
            blockers.append(f"{label} is locked but has no {field_name}")


def audit() -> dict[str, Any]:
    errors: list[str] = []
    blockers: list[str] = []

    try:
        mission = load_json(REGISTER_PATHS["mission"])
        geometry = load_json(REGISTER_PATHS["geometry"])
        loads = load_json(REGISTER_PATHS["loads"])
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        return {
            "schema_valid": False,
            "simulation_ready": False,
            "errors": [str(exc)],
            "blockers": [],
        }

    locked_mission = mission.get("locked_project_decisions", [])
    required_inputs = mission.get("required_inputs", [])
    locked_geometry = geometry.get("locked_parameters", [])
    geometry_parameters = geometry.get("parameters", [])
    load_cases = loads.get("load_cases", [])

    for name, value in (
        ("locked_project_decisions", locked_mission),
        ("required_inputs", required_inputs),
        ("locked_parameters", locked_geometry),
        ("parameters", geometry_parameters),
        ("load_cases", load_cases),
    ):
        if not isinstance(value, list):
            errors.append(f"{name} must be an array")

    if errors:
        return {
            "schema_valid": False,
            "simulation_ready": False,
            "errors": errors,
            "blockers": blockers,
        }

    known_input_ids = collect_unique_ids(
        [
            ("locked_project_decisions", locked_mission),
            ("required_inputs", required_inputs),
            ("locked_parameters", locked_geometry),
            ("geometry_parameters", geometry_parameters),
        ],
        errors,
    )
    collect_unique_ids([("load_cases", load_cases)], errors)

    for record in locked_mission:
        if record.get("status") != "locked" or not nonempty(record.get("value")):
            errors.append(f"locked mission decision {record.get('id')} is not locked with a value")

    for record in locked_geometry:
        if record.get("status") != "locked" or not nonempty(record.get("value")):
            errors.append(f"locked geometry parameter {record.get('id')} is not locked with a value")

    for record in required_inputs:
        require_locked(
            record,
            f"mission input {record.get('id')}",
            blockers,
            required_trace_fields=("authority", "evidence"),
        )

    for record in geometry_parameters:
        if record.get("required_for_simulation") is True:
            require_locked(
                record,
                f"geometry parameter {record.get('id')}",
                blockers,
                required_trace_fields=("source", "tolerance"),
            )
            if not nonempty(record.get("units")):
                errors.append(f"geometry parameter {record.get('id')} has no units")

    for case in load_cases:
        case_id = case.get("id", "unknown")
        if case.get("status") != "approved":
            blockers.append(f"load case {case_id} is not approved")
        elif not nonempty(case.get("approval_evidence")):
            blockers.append(f"load case {case_id} is approved but has no approval evidence")
        if not nonempty(case.get("boundary_condition")):
            errors.append(f"load case {case_id} has no boundary condition")
        if not nonempty(case.get("load_application")):
            errors.append(f"load case {case_id} has no load application")
        if not nonempty(case.get("required_outputs")):
            errors.append(f"load case {case_id} has no required outputs")
        if not nonempty(case.get("failure_modes")):
            errors.append(f"load case {case_id} has no failure modes")
        if not nonempty(case.get("acceptance_criteria")):
            blockers.append(f"load case {case_id} has no numeric acceptance criteria")

        for field_name in ("input_refs", "geometry_refs"):
            refs = case.get(field_name, [])
            if not isinstance(refs, list) or not refs:
                errors.append(f"load case {case_id} has no {field_name}")
                continue
            for ref in refs:
                if ref not in known_input_ids:
                    errors.append(f"load case {case_id} references unknown id {ref}")

    locked_values = {
        item.get("parameter"): item.get("value")
        for item in locked_mission
        if item.get("status") == "locked"
    }
    locked_geometry_values = {
        item.get("name"): item.get("value")
        for item in locked_geometry
        if item.get("status") == "locked"
    }
    if locked_values.get("height_mm") != locked_geometry_values.get("overall_height"):
        errors.append("height differs between mission and geometry registers")
    if locked_values.get("arm_span_mm") != locked_geometry_values.get("baseline_arm_span"):
        errors.append("arm span differs between mission and geometry registers")

    profile_path = PROJECT_DIR / "profiles" / "t56_domestic_frame.json"
    if profile_path.exists():
        try:
            profile = load_json(profile_path)
            if profile.get("height_mm") != locked_values.get("height_mm"):
                errors.append("mission height differs from t56_domestic_frame.json")
            if profile.get("arm_span_mm") != locked_values.get("arm_span_mm"):
                errors.append("mission arm span differs from t56_domestic_frame.json")
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            errors.append(f"cannot validate t56_domestic_frame.json: {exc}")

    blockers = sorted(set(blockers))
    errors = sorted(set(errors))
    simulation_ready = not errors and not blockers
    return {
        "project": "T56_CARBON",
        "schema_valid": not errors,
        "simulation_ready": simulation_ready,
        "counts": {
            "locked_project_decisions": len(locked_mission),
            "open_mission_inputs": sum(1 for item in required_inputs if item.get("status") != "locked"),
            "required_geometry_parameters": sum(1 for item in geometry_parameters if item.get("required_for_simulation") is True),
            "open_required_geometry_parameters": sum(
                1
                for item in geometry_parameters
                if item.get("required_for_simulation") is True and item.get("status") != "locked"
            ),
            "load_cases": len(load_cases),
            "approved_load_cases": sum(1 for item in load_cases if item.get("status") == "approved"),
        },
        "errors": errors,
        "blockers": blockers,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--strict",
        action="store_true",
        help="return exit status 2 while any simulation-readiness blocker remains",
    )
    parser.add_argument(
        "--compact",
        action="store_true",
        help="emit compact JSON instead of indented JSON",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    result = audit()
    if args.compact:
        print(json.dumps(result, sort_keys=True, separators=(",", ":")))
    else:
        print(json.dumps(result, indent=2, sort_keys=True))

    if result.get("errors"):
        return 1
    if args.strict and not result.get("simulation_ready"):
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
