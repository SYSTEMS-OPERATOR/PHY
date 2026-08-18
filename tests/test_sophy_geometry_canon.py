from __future__ import annotations

import json
import math
from dataclasses import replace
from pathlib import Path
import tempfile
import unittest

from skeleton.canon import (
    PHI,
    SOPHY_GEOMETRY_CANON_VERSION,
    CanonIntegrityError,
    CanonicalGeometry,
    EmbodimentOverlay,
    MeasuredLandmarkDeviation,
    OverlayMutationError,
    Point3,
    golden_major,
    golden_minor,
    instantiate_sophy_canon,
)
from skeleton.exporters.canon_exporter import CanonicalGeometryExporter
from skeleton.validation.canon_validator import CanonicalGeometryValidator


class SophyGeometryCanonTest(unittest.TestCase):
    def test_same_height_produces_identical_geometry(self):
        first = instantiate_sophy_canon(1676.4)
        second = instantiate_sophy_canon(1676.4)
        self.assertEqual(first.to_dict(), second.to_dict())
        self.assertEqual(first.to_json(), second.to_json())

    def test_different_heights_are_geometrically_similar(self):
        small = instantiate_sophy_canon(1676.4)
        large = instantiate_sophy_canon(2540.0)
        for small_landmark in small.landmarks:
            large_landmark = large.landmark(small_landmark.canonical_id)
            self.assertEqual(small_landmark.normalized, large_landmark.normalized)
            for small_coord, large_coord in zip(
                small_landmark.coordinates_mm.as_list(),
                large_landmark.coordinates_mm.as_list(),
            ):
                self.assertAlmostEqual(small_coord / small.height_mm, large_coord / large.height_mm)

    def test_bilateral_symmetry_is_exact_reflection(self):
        geometry = instantiate_sophy_canon(1676.4)
        right = geometry.landmark("construction.arm_span_endpoint.right")
        left = geometry.landmark("construction.arm_span_endpoint.left")
        self.assertEqual(left.coordinates_mm, right.coordinates_mm.mirror_x())
        self.assertEqual(left.normalized, right.normalized.mirror_x())
        self.assertEqual(left.mirrored_from, right.canonical_id)

    def test_arm_span_equals_height(self):
        geometry = instantiate_sophy_canon(1676.4)
        left = geometry.landmark("construction.arm_span_endpoint.left").coordinates_mm
        right = geometry.landmark("construction.arm_span_endpoint.right").coordinates_mm
        self.assertEqual(geometry.arm_span_mm, geometry.height_mm)
        self.assertEqual(right.x - left.x, geometry.height_mm)

    def test_midline_landmarks_remain_centered(self):
        geometry = instantiate_sophy_canon(1676.4)
        for landmark in geometry.landmarks:
            if landmark.side == "MIDLINE":
                self.assertEqual(landmark.coordinates_mm.x, 0.0)

    def test_phi_is_named_and_not_an_anatomical_binding(self):
        geometry = instantiate_sophy_canon(1000.0)
        self.assertEqual(PHI, (1.0 + math.sqrt(5.0)) / 2.0)
        self.assertAlmostEqual(golden_major(1000.0) + golden_minor(1000.0), 1000.0)
        for operator in geometry.to_dict()["proportional_operators"].values():
            self.assertIsNone(operator["identity_binding"])

    def test_project_overlay_cannot_mutate_canon(self):
        geometry = instantiate_sophy_canon(1676.4)
        original = geometry.landmark("construction.arm_span_endpoint.right").coordinates_mm
        overlay = EmbodimentOverlay(
            project_id="TEST_ARTICLE",
            canon=geometry,
            deviations=(
                MeasuredLandmarkDeviation(
                    landmark_id="construction.arm_span_endpoint.right",
                    delta_mm=Point3(0.2, -0.1, 0.3),
                    reason="post-build metrology",
                    evidence_id="METROLOGY-001",
                ),
            ),
        )
        self.assertNotEqual(overlay.resolved_landmark("construction.arm_span_endpoint.right"), original)
        self.assertEqual(geometry.landmark("construction.arm_span_endpoint.right").coordinates_mm, original)
        with self.assertRaises(OverlayMutationError):
            EmbodimentOverlay.from_payload(
                geometry,
                {
                    "project_id": "TEST_ARTICLE",
                    "canon_reference": overlay.to_dict()["canon_reference"],
                    "landmarks": [],
                },
            )

    def test_export_import_round_trip_preserves_geometry(self):
        geometry = instantiate_sophy_canon(1676.4)
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "sophy_canonical_geometry.json"
            CanonicalGeometryExporter(geometry).export(path)
            payload = json.loads(path.read_text(encoding="utf-8"))
        imported = CanonicalGeometry.from_dict(payload)
        self.assertEqual(imported, geometry)

    def test_import_rejects_identity_drift(self):
        geometry = instantiate_sophy_canon(1676.4)
        payload = geometry.to_dict()
        payload["canonical_dimensions_mm"]["arm_span"] += 1.0
        with self.assertRaises(CanonIntegrityError):
            CanonicalGeometry.from_dict(payload)

    def test_canon_revision_is_explicit_and_validated(self):
        geometry = instantiate_sophy_canon(1676.4)
        self.assertEqual(geometry.canon_version, SOPHY_GEOMETRY_CANON_VERSION)
        report = CanonicalGeometryValidator(geometry).run()
        self.assertTrue(report["summary"]["pass"], report)
        with self.assertRaises(ValueError):
            instantiate_sophy_canon(1676.4, canon_version="0.0.0")

    def test_validator_reports_missing_landmark_without_crashing(self):
        geometry = instantiate_sophy_canon(1676.4)
        broken = replace(
            geometry,
            landmarks=tuple(
                landmark
                for landmark in geometry.landmarks
                if landmark.canonical_id != "body.canonical_center"
            ),
        )
        report = CanonicalGeometryValidator(broken).run()
        self.assertFalse(report["summary"]["pass"])
        self.assertIn("unresolved_required_landmark", {row["code"] for row in report["issues"]})


if __name__ == "__main__":
    unittest.main()
