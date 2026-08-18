"""Versioned, project-independent SOPHY identity geometry."""

from .geometry import (
    PHI,
    SOPHY_GEOMETRY_CANON_VERSION,
    CanonIntegrityError,
    CanonicalGeometry,
    CanonicalLandmark,
    Point3,
    golden_major,
    golden_minor,
    golden_recurse,
    instantiate_sophy_canon,
)
from .overlay import EmbodimentOverlay, MeasuredLandmarkDeviation, OverlayMutationError

__all__ = [
    "PHI",
    "SOPHY_GEOMETRY_CANON_VERSION",
    "CanonIntegrityError",
    "CanonicalGeometry",
    "CanonicalLandmark",
    "EmbodimentOverlay",
    "MeasuredLandmarkDeviation",
    "OverlayMutationError",
    "Point3",
    "golden_major",
    "golden_minor",
    "golden_recurse",
    "instantiate_sophy_canon",
]
