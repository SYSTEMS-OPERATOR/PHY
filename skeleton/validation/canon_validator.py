"""Machine-detectable invariants for SOPHY canonical geometry."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Dict, List, Tuple

from skeleton.canon.geometry import CanonicalGeometry, instantiate_sophy_canon


_SUPPORTED_DERIVATION_OPERATORS = frozenset(
    {"identity", "mirror_x", "scale_normalized_by_H"}
)


@dataclass(frozen=True, slots=True)
class CanonicalGeometryValidator:
    """Validate identity laws independently of fabrication embodiment."""

    geometry: CanonicalGeometry

    def run(self) -> Dict[str, Any]:
        issues: List[Dict[str, Any]] = []
        height = self.geometry.height_mm
        tolerance = max(1e-9, height * 1e-12)

        def fail(code: str, path: str, message: str) -> None:
            issues.append({"code": code, "path": path, "message": message})

        try:
            expected = instantiate_sophy_canon(
                height,
                canon_version=self.geometry.canon_version,
            )
        except ValueError as exc:
            expected = None
            fail("unsupported_canon_revision", "canon.version", str(exc))

        if expected is not None and self.geometry.canon_law_hash != expected.canon_law_hash:
            fail(
                "canon_law_hash_mismatch",
                "canon.law_hash_sha256",
                "law hash does not match the executable canon revision",
            )

        if not math.isclose(self.geometry.arm_span_mm, height, rel_tol=0.0, abs_tol=tolerance):
            fail("arm_span_height_mismatch", "canonical_dimensions_mm.arm_span", "arm span must equal H")
        if not math.isclose(self.geometry.standing_height_mm, height, rel_tol=0.0, abs_tol=tolerance):
            fail(
                "standing_height_mismatch",
                "canonical_dimensions_mm.standing_height",
                "standing height must equal H",
            )

        ids = [landmark.canonical_id for landmark in self.geometry.landmarks]
        duplicates = sorted({item for item in ids if ids.count(item) > 1})
        for duplicate in duplicates:
            fail("duplicate_landmark", f"landmarks.{duplicate}", "canonical landmark IDs must be unique")
        if expected is not None:
            expected_ids = {landmark.canonical_id for landmark in expected.landmarks}
            actual_ids = set(ids)
            for missing in sorted(expected_ids.difference(actual_ids)):
                fail("unresolved_required_landmark", f"landmarks.{missing}", "required canon landmark is missing")
            for unexpected in sorted(actual_ids.difference(expected_ids)):
                fail(
                    "unsupported_canonical_landmark",
                    f"landmarks.{unexpected}",
                    "landmark is not defined by this canon revision",
                )

        for landmark in self.geometry.landmarks:
            path = f"landmarks.{landmark.canonical_id}"
            if landmark.derivation_operator not in _SUPPORTED_DERIVATION_OPERATORS:
                fail(
                    "unsupported_derivation_operator",
                    f"{path}.derivation.operator",
                    f"unsupported operator {landmark.derivation_operator!r}",
                )
            expected_point = landmark.normalized.scaled(height)
            for axis, actual, target in zip(
                "xyz",
                landmark.coordinates_mm.as_list(),
                expected_point.as_list(),
            ):
                if not math.isclose(actual, target, rel_tol=0.0, abs_tol=tolerance):
                    fail(
                        "scaling_inconsistency",
                        f"{path}.coordinates_mm.{axis}",
                        "coordinate does not equal normalized coordinate multiplied by H",
                    )
            if landmark.side == "MIDLINE" and not math.isclose(
                landmark.coordinates_mm.x, 0.0, rel_tol=0.0, abs_tol=tolerance
            ):
                fail("midline_drift", f"{path}.coordinates_mm.x", "midline landmark must satisfy x = 0")

            if landmark.side == "LEFT":
                if landmark.mirrored_from is None:
                    fail("independent_left_landmark", path, "left landmark must name its right-side source")
                    continue
                try:
                    right = self.geometry.landmark(landmark.mirrored_from)
                except KeyError:
                    fail("broken_mirror_dependency", path, "mirrored right-side landmark does not exist")
                    continue
                mirrored = right.coordinates_mm.mirror_x()
                for axis, actual, target in zip(
                    "xyz",
                    landmark.coordinates_mm.as_list(),
                    mirrored.as_list(),
                ):
                    if not math.isclose(actual, target, rel_tol=0.0, abs_tol=tolerance):
                        fail(
                            "bilateral_symmetry_violation",
                            f"{path}.coordinates_mm.{axis}",
                            "left landmark is not the exact midsagittal reflection of right",
                        )

        try:
            left = self.geometry.landmark("construction.arm_span_endpoint.left")
            right = self.geometry.landmark("construction.arm_span_endpoint.right")
            measured_span = right.coordinates_mm.x - left.coordinates_mm.x
            if not math.isclose(measured_span, height, rel_tol=0.0, abs_tol=tolerance):
                fail(
                    "arm_span_endpoint_mismatch",
                    "landmarks.construction.arm_span_endpoint",
                    "endpoint separation must equal H",
                )
        except KeyError as exc:
            fail("unresolved_required_landmark", "landmarks", f"missing {exc.args[0]}")

        try:
            center = self.geometry.landmark("body.canonical_center").coordinates_mm
        except KeyError:
            center = None
        for landmark_id in (
            "body.plantar_midpoint",
            "cranial.vertex",
            "construction.arm_span_endpoint.left",
            "construction.arm_span_endpoint.right",
        ):
            if center is None:
                break
            try:
                point = self.geometry.landmark(landmark_id).coordinates_mm
            except KeyError:
                continue
            radial_distance = math.hypot(point.x - center.x, point.z - center.z)
            if not math.isclose(
                radial_distance,
                self.geometry.circle_radius_mm,
                rel_tol=0.0,
                abs_tol=tolerance,
            ):
                fail(
                    "canonical_circle_violation",
                    f"landmarks.{landmark_id}",
                    "cardinal construction landmark is not on the canonical circle",
                )

        return {
            "summary": {
                "pass": not issues,
                "canon_version": self.geometry.canon_version,
                "height_mm": height,
                "issue_count": len(issues),
                "numerical_tolerance_mm": tolerance,
            },
            "issues": issues,
        }

    @staticmethod
    def write_report(
        report: Dict[str, Any],
        path: Path = Path("reports/sophy_canon_validation_report.json"),
    ) -> Path:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return path
