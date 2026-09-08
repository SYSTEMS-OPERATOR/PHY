#!/usr/bin/env python3
"""Audit T-5.6 mission, geometry, and load-case convergence registers."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

# Support direct execution from any working directory, including off-grid use.
REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from skeleton.canon import instantiate_sophy_canon
from PROJECTS.T56_CARBON.tools.geometry_contracts import (
    CONTRACTS, FRAME_ID, finite_number, text, validate_record,
)

PROJECT_DIR = Path(__file__).resolve().parents[1]
REQUIREMENTS_DIR = PROJECT_DIR / "requirements"
REGISTER_PATHS = {
    "mission": REQUIREMENTS_DIR / "mission_envelope.json",
    "geometry": REQUIREMENTS_DIR / "geometry_register.json",
    "loads": REQUIREMENTS_DIR / "load_case_register.json",
}
PROFILE_PATHS = {
    "single-side": REQUIREMENTS_DIR / "single_side_article_closure.json",
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


def audit(profile_name: str | None = None) -> dict[str, Any]:
    errors: list[str] = []
    blockers: list[str] = []

    try:
        mission = load_json(REGISTER_PATHS["mission"])
        geometry = load_json(REGISTER_PATHS["geometry"])
        loads = load_json(REGISTER_PATHS["loads"])
        closure_profile = load_json(PROFILE_PATHS[profile_name]) if profile_name else None
    except (OSError, ValueError, KeyError) as exc:
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
        if not isinstance(value, list) or not value or not all(isinstance(row, dict) for row in value):
            errors.append(f"{name} must be a non-empty array of objects")

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

    # Invalid IDs must not reach dictionary/set indexing below.
    if errors:
        return {"schema_valid": False, "scope_ready": False, "errors": errors, "blockers": []}

    geometry_ids_present = {item["id"] for item in geometry_parameters}
    for missing in sorted(set(CONTRACTS) - geometry_ids_present):
        errors.append(f"missing required geometry parameter {missing}")
    for record in geometry_parameters:
        errors.extend(validate_record(record))

    frame = geometry.get("coordinate_frame")
    expected_axes = {"x": "left_to_right", "y": "posterior_to_anterior", "z": "inferior_to_superior"}
    if not isinstance(frame, dict) or frame.get("frame_id") != FRAME_ID or frame.get("axes") != expected_axes:
        errors.append("geometry coordinate frame must retain the declared T56 XYZ convention")
    elif frame.get("dimensioned_datum_status") != "locked":
        blockers.append("geometry datum has no dimensioned fixture realization")
    elif (
        frame.get("definition_status") != "locked"
        or frame.get("handedness") != "right_handed"
        or not text(frame.get("origin_definition"))
        or not text(frame.get("datum_revision"))
        or not text(frame.get("dimensioned_datum_evidence"))
    ):
        blockers.append("locked geometry datum requires verified handedness, origin, revision, and evidence")

    for case in load_cases:
        for field in ("input_refs", "geometry_refs"):
            refs = case.get(field)
            if not isinstance(refs, list) or not refs or not all(text(ref) for ref in refs):
                errors.append(f"load case {case['id']} {field} must be a non-empty array of IDs")
    if errors:
        return {"schema_valid": False, "scope_ready": False, "errors": sorted(set(errors)), "blockers": blockers}

    required_inputs_to_gate = required_inputs
    geometry_to_gate = [
        item for item in geometry_parameters
        if item.get("required_for_simulation") is True
    ]
    load_case_ids_to_gate = {item.get("id") for item in load_cases}
    evidence_packages: list[dict[str, Any]] = []

    if closure_profile is not None:
        mission_ids = closure_profile.get("required_mission_inputs", [])
        geometry_ids = closure_profile.get("required_geometry_parameters", [])
        load_case_ids = closure_profile.get("required_load_cases", [])
        evidence_packages = closure_profile.get("article_evidence_packages", [])

        for field_name, ids in (
            ("required_mission_inputs", mission_ids),
            ("required_geometry_parameters", geometry_ids),
            ("required_load_cases", load_case_ids),
        ):
            if not isinstance(ids, list) or not ids or not all(text(item) for item in ids) or len(ids) != len(set(ids)):
                errors.append(f"closure profile {field_name} must be a non-empty unique array")

        if not isinstance(evidence_packages, list) or not evidence_packages or not all(isinstance(row, dict) for row in evidence_packages):
            errors.append("closure profile article_evidence_packages must be a non-empty array of objects")
        else:
            collect_unique_ids([("article_evidence_packages", evidence_packages)], errors)
        if errors:
            return {"schema_valid": False, "scope_ready": False, "errors": errors, "blockers": blockers}

        mission_by_id = {item.get("id"): item for item in required_inputs}
        geometry_by_id = {item.get("id"): item for item in geometry_parameters}
        load_by_id = {item.get("id"): item for item in load_cases}

        for record_id in mission_ids:
            if record_id not in mission_by_id:
                errors.append(f"closure profile references unknown mission input {record_id}")
        for record_id in geometry_ids:
            if record_id not in geometry_by_id:
                errors.append(f"closure profile references unknown geometry parameter {record_id}")
        for record_id in load_case_ids:
            if record_id not in load_by_id:
                errors.append(f"closure profile references unknown load case {record_id}")

        selected_cases = [load_by_id[item] for item in load_case_ids if item in load_by_id]
        assigned_cases = {case["id"] for case in load_cases if case.get("article") == closure_profile.get("article")}
        if set(load_case_ids) != assigned_cases:
            errors.append("closure profile must include every load case assigned to its article")
        derived_mission_ids = {
            ref for case in selected_cases for ref in case.get("input_refs", [])
            if isinstance(ref, str) and ref.startswith("ME-")
        }
        derived_geometry_ids = {
            ref for case in selected_cases for ref in case.get("geometry_refs", [])
        }
        if set(mission_ids) != derived_mission_ids:
            errors.append(
                "closure profile mission inputs must exactly equal the selected load-case dependency union"
            )
        if set(geometry_ids) != derived_geometry_ids:
            errors.append(
                "closure profile geometry parameters must exactly equal the selected load-case dependency union"
            )
        if any(case.get("article") != closure_profile.get("article") for case in selected_cases):
            errors.append("closure profile includes a load case assigned to another article")
        if not isinstance(evidence_packages, list) or not evidence_packages:
            errors.append("closure profile article_evidence_packages must be a non-empty array")

        required_inputs_to_gate = [
            mission_by_id[item] for item in mission_ids if item in mission_by_id
        ]
        geometry_to_gate = [
            geometry_by_id[item] for item in geometry_ids if item in geometry_by_id
        ]
        load_case_ids_to_gate = set(load_case_ids)

    for record in locked_mission:
        if record.get("status") != "locked" or not nonempty(record.get("value")):
            errors.append(f"locked mission decision {record.get('id')} is not locked with a value")

    for record in locked_geometry:
        if record.get("status") != "locked" or not nonempty(record.get("value")):
            errors.append(f"locked geometry parameter {record.get('id')} is not locked with a value")

    for record in required_inputs_to_gate:
        require_locked(
            record,
            f"mission input {record.get('id')}",
            blockers,
            required_trace_fields=("authority", "evidence"),
        )

    for record in geometry_to_gate:
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
        if case_id in load_case_ids_to_gate:
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
        if case_id in load_case_ids_to_gate and not nonempty(case.get("acceptance_criteria")):
            blockers.append(f"load case {case_id} has no numeric acceptance criteria")

        for field_name in ("input_refs", "geometry_refs"):
            refs = case.get(field_name, [])
            if not isinstance(refs, list) or not refs:
                errors.append(f"load case {case_id} has no {field_name}")
                continue
            for ref in refs:
                if ref not in known_input_ids:
                    errors.append(f"load case {case_id} references unknown id {ref}")

    for package in evidence_packages:
        package_id = package.get("id", "unknown")
        if package.get("status") != "approved":
            blockers.append(f"article evidence package {package_id} is not approved")
        elif not nonempty(package.get("evidence_refs")):
            blockers.append(f"article evidence package {package_id} has no evidence references")
        if not nonempty(package.get("required_contents")):
            errors.append(f"article evidence package {package_id} has no required contents")

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
    try:
        profile = load_json(profile_path)
        reference = profile.get("canon_reference")
        if not isinstance(reference, dict) or not finite_number(reference.get("height_mm")):
            raise ValueError("canon_reference requires a finite numeric height_mm")
        if not text(reference.get("canon_version")):
            raise ValueError("canon_reference requires an explicit canon_version")
        canon = instantiate_sophy_canon(reference["height_mm"], canon_version=reference["canon_version"])
        for name, expected in (("height_mm", canon.height_mm), ("arm_span_mm", canon.arm_span_mm)):
            if not finite_number(profile.get(name)) or profile.get(name) != expected:
                errors.append(f"profile {name} differs from bound SOPHY canon")
            if not finite_number(locked_values.get(name)) or locked_values.get(name) != expected:
                errors.append(f"mission {name} differs from bound SOPHY canon")
        for name, expected in (("overall_height", canon.height_mm), ("baseline_arm_span", canon.arm_span_mm)):
            if not finite_number(locked_geometry_values.get(name)) or locked_geometry_values.get(name) != expected:
                errors.append(f"geometry {name} differs from bound SOPHY canon")
        for record in locked_geometry + [row for row in locked_mission if row.get("parameter") in ("height_mm", "arm_span_mm")]:
            if record.get("units") != "mm":
                errors.append(f"locked scale {record.get('id')} must use mm")
    except (OSError, ValueError, TypeError) as exc:
        errors.append(f"cannot validate t56_domestic_frame.json: {exc}")

    blockers = sorted(set(blockers))
    errors = sorted(set(errors))
    scope_ready = not errors and not blockers
    result = {
        "project": "T56_CARBON",
        "scope": closure_profile.get("profile_id") if closure_profile else "full_system",
        "schema_valid": not errors,
        "scope_ready": scope_ready,
        "counts": {
            "locked_project_decisions": len(locked_mission),
            "mission_inputs_in_scope": len(required_inputs_to_gate),
            "open_mission_inputs_in_scope": sum(
                1 for item in required_inputs_to_gate if item.get("status") != "locked"
            ),
            "geometry_parameters_in_scope": len(geometry_to_gate),
            "open_geometry_parameters_in_scope": sum(
                1 for item in geometry_to_gate if item.get("status") != "locked"
            ),
            "load_cases_in_scope": len(load_case_ids_to_gate),
            "approved_load_cases_in_scope": sum(
                1 for item in load_cases
                if item.get("id") in load_case_ids_to_gate and item.get("status") == "approved"
            ),
            "article_evidence_packages_in_scope": len(evidence_packages),
            "approved_article_evidence_packages": sum(
                1 for item in evidence_packages if item.get("status") == "approved"
            ),
        },
        "errors": errors,
        "blockers": blockers,
    }
    if closure_profile:
        result["article_ready"] = scope_ready
    else:
        result["simulation_ready"] = scope_ready
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--strict",
        action="store_true",
        help="return exit status 2 while any simulation-readiness blocker remains",
    )
    parser.add_argument(
        "--profile",
        choices=sorted(PROFILE_PATHS),
        help="audit only the dependencies and evidence for a named closure profile",
    )
    parser.add_argument(
        "--compact",
        action="store_true",
        help="emit compact JSON instead of indented JSON",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    result = audit(args.profile)
    if args.compact:
        print(json.dumps(result, sort_keys=True, separators=(",", ":")))
    else:
        print(json.dumps(result, indent=2, sort_keys=True))

    if result.get("errors"):
        return 1
    if args.strict and not result.get("scope_ready"):
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
