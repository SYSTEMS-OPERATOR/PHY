"""Typed engineering geometry inputs; no dimensions or canonical laws are supplied.

Coordinates describe the project-local FRAME_T56_THORAX, not anatomical canon
landmarks. Numerical validation is necessary, never sufficient, for release.
"""

from __future__ import annotations

import math
from typing import Any


FRAME_ID = "FRAME_T56_THORAX"
# ID -> (value shape, register units). Adding a parameter requires a contract.
CONTRACTS = {
    "GEO-THX-001": ("length", "mm"),
    "GEO-THX-002": ("length", "mm"),
    "GEO-THX-003": ("length", "mm"),
    "GEO-THX-004": ("offset", "mm"),
    "GEO-PEL-001": ("length", "mm"),
    "GEO-PEL-002": ("transform", "mm_and_deg"),
    "GEO-PEL-003": ("point", "mm"),
    "GEO-PEL-004": ("point", "mm"),
    "GEO-SHO-001": ("point", "mm"),
    "GEO-SHO-002": ("point", "mm"),
    "GEO-SHO-003": ("guide", "mm_and_unit_vector"),
    "GEO-SHO-004": ("points", "mm"),
    "GEO-SHO-005": ("trajectory", "mm_and_deg_by_pose"),
    "GEO-SHO-006": ("point", "mm"),
    "GEO-SHO-007": ("joints", "unit_vectors_and_deg"),
    "GEO-ARM-001": ("length", "mm"),
    "GEO-CLR-001": ("aabb", "mm"),
    "GEO-CLR-002": ("aabb", "mm"),
    "GEO-CLR-003": ("aabb", "mm"),
}


def finite_number(value: Any) -> bool:
    # bool is an int subclass; it is not an engineering measurement.
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def text(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip())


def vector(value: Any, size: int = 3) -> bool:
    return isinstance(value, list) and len(value) == size and all(map(finite_number, value))


def fields(value: Any, names: set[str]) -> bool:
    return isinstance(value, dict) and set(value) == names


def unit_vector(value: Any) -> bool:
    return vector(value) and math.isclose(math.hypot(*value), 1.0, rel_tol=0, abs_tol=1e-9)


def ordered_pair(value: Any) -> bool:
    return vector(value, 2) and value[0] < value[1]


def points(value: Any) -> bool:
    return (
        isinstance(value, list) and len(value) >= 2 and all(map(vector, value))
        and all(a != b for a, b in zip(value, value[1:]))
    )


def named_rows(value: Any, name: str) -> bool:
    if not isinstance(value, list) or not value:
        return False
    if not all(isinstance(row, dict) and text(row.get(name)) for row in value):
        return False
    return len({row[name] for row in value}) == len(value)


def valid_value(kind: str, value: Any) -> bool:
    if kind in ("length", "offset"):
        return finite_number(value) and (kind == "offset" or value > 0)
    if kind == "point":
        return vector(value)
    if kind == "points":
        return points(value)
    if kind == "transform":
        return (
            fields(value, {"child_frame_id", "translation_mm", "rpy_deg"})
            and text(value["child_frame_id"]) and value["child_frame_id"] != FRAME_ID
            and vector(value["translation_mm"]) and vector(value["rpy_deg"])
        )
    if kind == "guide":
        if not isinstance(value, dict):
            return False
        if value.get("type") == "line":
            return (
                fields(value, {"type", "origin_mm", "direction_unit", "travel_mm"})
                and vector(value["origin_mm"]) and unit_vector(value["direction_unit"])
                and ordered_pair(value["travel_mm"])
            )
        return (
            fields(value, {"type", "points_mm"}) and value["type"] == "polyline"
            and points(value["points_mm"])
        )
    if kind == "trajectory":
        return named_rows(value, "pose_id") and len(value) >= 2 and all(
            fields(row, {"pose_id", "position_mm", "rpy_deg"})
            and vector(row["position_mm"]) and vector(row["rpy_deg"])
            for row in value
        )
    if kind == "joints":
        return named_rows(value, "joint_id") and all(
            fields(row, {"joint_id", "origin_mm", "axis_unit", "range_deg"})
            and vector(row["origin_mm"]) and unit_vector(row["axis_unit"])
            and ordered_pair(row["range_deg"])
            for row in value
        )
    if kind == "aabb":
        return (
            fields(value, {"min_mm", "max_mm"})
            and vector(value["min_mm"]) and vector(value["max_mm"])
            and all(lo < hi for lo, hi in zip(value["min_mm"], value["max_mm"]))
        )
    return False


def validate_record(record: dict[str, Any]) -> list[str]:
    """Validate supplied values, including provisional ones, without filling gaps."""
    record_id = record.get("id")
    if record_id not in CONTRACTS:
        return [f"geometry parameter {record_id} has no typed geometry contract"]
    kind, units = CONTRACTS[record_id]
    errors = []
    label = f"geometry parameter {record_id}"
    if record.get("units") != units:
        errors.append(f"{label} must use {units}")
    if record.get("required_for_simulation") is not True:
        errors.append(f"{label} cannot opt out of the simulation geometry gate")
    if record.get("status") not in ("open", "locked"):
        errors.append(f"{label} status must be open or locked")
    value = record.get("value")
    if value is None and record.get("status") == "open":
        return errors
    if not valid_value(kind, value):
        errors.append(f"{label} value must satisfy {kind} contract (finite numbers, exact shape)")
    if kind not in ("length", "offset") and record.get("frame_id") != FRAME_ID:
        errors.append(f"{label} must explicitly reference {FRAME_ID}")
    if record.get("status") == "locked":
        if not text(record.get("source")):
            errors.append(f"{label} must have a non-blank traceable source")
        tolerance = record.get("tolerance")
        keys = {"linear_mm"}
        if kind in ("transform", "trajectory", "joints"):
            keys.add("angular_deg")
        if not fields(tolerance, keys) or not all(
            finite_number(v) and v > 0 for v in tolerance.values()
        ):
            errors.append(f"{label} tolerance requires positive finite {', '.join(sorted(keys))}")
    return errors
