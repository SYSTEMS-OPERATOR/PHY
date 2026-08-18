"""Project embodiment overlays that cannot mutate SOPHY identity geometry."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Iterable, Mapping, Tuple

from .geometry import CanonicalGeometry, Point3


class OverlayMutationError(ValueError):
    """Raised when an overlay attempts to overwrite canonical authority."""


_PROTECTED_OVERLAY_KEYS = frozenset(
    {
        "canon",
        "canonical_dimensions_mm",
        "canonical_landmarks",
        "coordinate_system",
        "landmarks",
        "midsagittal_plane",
        "symmetry",
    }
)


@dataclass(frozen=True, slots=True)
class MeasuredLandmarkDeviation:
    """Measured article deviation from canon, never a replacement canon point."""

    landmark_id: str
    delta_mm: Point3
    reason: str
    evidence_id: str

    def __post_init__(self) -> None:
        if not self.landmark_id:
            raise ValueError("landmark_id is required")
        if not self.reason:
            raise ValueError("reason is required")
        if not self.evidence_id:
            raise ValueError("evidence_id is required")

    def to_dict(self) -> Dict[str, Any]:
        return {
            "landmark_id": self.landmark_id,
            "classification": "measured_deviation_from_canon",
            "delta_mm": self.delta_mm.as_list(),
            "reason": self.reason,
            "evidence_id": self.evidence_id,
        }


@dataclass(frozen=True, slots=True)
class EmbodimentOverlay:
    """Immutable project overlay bound to one canon instance and law hash."""

    project_id: str
    canon: CanonicalGeometry
    deviations: Tuple[MeasuredLandmarkDeviation, ...] = ()

    def __post_init__(self) -> None:
        if not self.project_id:
            raise ValueError("project_id is required")
        seen = set()
        for deviation in self.deviations:
            self.canon.landmark(deviation.landmark_id)
            if deviation.landmark_id in seen:
                raise ValueError(f"duplicate deviation for {deviation.landmark_id}")
            seen.add(deviation.landmark_id)

    def resolved_landmark(self, canonical_id: str) -> Point3:
        point = self.canon.landmark(canonical_id).coordinates_mm
        for deviation in self.deviations:
            if deviation.landmark_id == canonical_id:
                return point.plus(deviation.delta_mm)
        return point

    def to_dict(self) -> Dict[str, Any]:
        return {
            "artifact": "SOPHY_EMBODIMENT_OVERLAY",
            "project_id": self.project_id,
            "canon_reference": {
                "version": self.canon.canon_version,
                "law_hash_sha256": self.canon.canon_law_hash,
                "height_mm": self.canon.height_mm,
            },
            "measured_landmark_deviations": [row.to_dict() for row in self.deviations],
        }

    @classmethod
    def from_payload(
        cls,
        canon: CanonicalGeometry,
        payload: Mapping[str, Any],
    ) -> "EmbodimentOverlay":
        forbidden = _PROTECTED_OVERLAY_KEYS.intersection(payload)
        if forbidden:
            names = ", ".join(sorted(forbidden))
            raise OverlayMutationError(f"overlay attempts to overwrite canon fields: {names}")
        allowed = {
            "artifact",
            "project_id",
            "canon_reference",
            "measured_landmark_deviations",
        }
        unknown = set(payload).difference(allowed)
        if unknown:
            names = ", ".join(sorted(unknown))
            raise OverlayMutationError(f"unsupported overlay fields: {names}")

        reference = payload.get("canon_reference", {})
        expected_reference = {
            "version": canon.canon_version,
            "law_hash_sha256": canon.canon_law_hash,
            "height_mm": canon.height_mm,
        }
        if reference != expected_reference:
            raise OverlayMutationError("overlay canon reference does not match the supplied canon")

        deviations = []
        rows: Iterable[Mapping[str, Any]] = payload.get("measured_landmark_deviations", [])
        for row in rows:
            if row.get("classification") != "measured_deviation_from_canon":
                raise OverlayMutationError("deviation classification must be measured_deviation_from_canon")
            deviations.append(
                MeasuredLandmarkDeviation(
                    landmark_id=str(row["landmark_id"]),
                    delta_mm=Point3.from_iterable(row["delta_mm"]),
                    reason=str(row["reason"]),
                    evidence_id=str(row["evidence_id"]),
                )
            )
        return cls(
            project_id=str(payload.get("project_id", "")),
            canon=canon,
            deviations=tuple(deviations),
        )
