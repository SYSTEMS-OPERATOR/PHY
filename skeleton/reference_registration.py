"""Rigid reference landmark fit with explicit units and reference-only output.

Four noncoplanar correspondences determine handedness. Scale is fixed by units;
there is no fitted body scale, canon adoption, joint-center inference or release.
"""
from __future__ import annotations
import math
import numpy as np


def register_landmarks(packet):
    if not isinstance(packet, dict) or set(packet) != {"source", "target", "max_residual_mm"}:
        raise ValueError("Expected source, target and reviewed residual limit")
    limit = packet["max_residual_mm"]
    if type(limit) not in (int, float) or not math.isfinite(limit) or limit <= 0:
        raise ValueError("Invalid reviewed residual limit")
    arrays = []
    for label in ("source", "target"):
        p = packet[label]
        if not isinstance(p, dict) or set(p) != {"frame", "units", "points"}:
            raise ValueError("Invalid " + label + " landmark set")
        if not isinstance(p["frame"], str) or not p["frame"].strip():
            raise ValueError("Missing explicit frame")
        if p["units"] not in ("mm", "m"):
            raise ValueError("Expected mm or m")
        if not isinstance(p["points"], dict) or len(p["points"]) < 4:
            raise ValueError("Four named noncoplanar correspondences are required")
        if any(not isinstance(k, str) or not k.strip() for k in p["points"]):
            raise ValueError("Invalid landmark name")
        keys = sorted(p["points"])
        values = [p["points"][k] for k in keys]
        if any(not isinstance(v, list) or len(v) != 3
               or any(type(x) not in (int, float) or not math.isfinite(x) for x in v) for v in values):
            raise ValueError("Invalid finite landmark coordinate")
        with np.errstate(over="ignore", invalid="ignore"):
            a = np.asarray(values, dtype=float)*(1000 if p["units"] == "m" else 1)
        if not np.isfinite(a).all() or np.abs(a).max() > 1e9:
            raise ValueError("Landmark coordinate outside bounded reference budget")
        centered = a-a.mean(axis=0)
        singular = np.linalg.svd(centered, compute_uv=False)
        if singular[-1] <= singular[0]*1e-8:
            raise ValueError("Coplanar or degenerate landmark configuration")
        arrays.append(a)
    if sorted(packet["source"]["points"]) != sorted(packet["target"]["points"]):
        raise ValueError("Source/target landmark names differ")
    if packet["source"]["frame"] == packet["target"]["frame"]:
        raise ValueError("Source and target frames must be distinct")
    a, b = arrays
    u, _, vh = np.linalg.svd((a-a.mean(axis=0)).T @ (b-b.mean(axis=0)))
    rotation = vh.T @ u.T
    if np.linalg.det(rotation) < 0:
        vh[-1] *= -1
        rotation = vh.T @ u.T
    translation = b.mean(axis=0)-rotation @ a.mean(axis=0)
    fitted = a @ rotation.T + translation
    residuals = np.linalg.norm(fitted-b, axis=1)
    maximum = float(residuals.max())
    if maximum > limit:
        raise ValueError("Landmark fit exceeds reviewed residual limit; reflection/scale/annotation requires review")
    return {"version": "1.0.0", "source_frame": packet["source"]["frame"],
            "target_frame": packet["target"]["frame"], "matrix_units": "input and output normalized to mm",
            "source_units": packet["source"]["units"], "target_units": packet["target"]["units"],
            "source_to_mm_scale": 1000 if packet["source"]["units"] == "m" else 1,
            "target_to_mm_scale": 1000 if packet["target"]["units"] == "m" else 1,
            "rotation": rotation.tolist(), "translation_mm": translation.tolist(),
            "determinant": float(np.linalg.det(rotation)), "fitted_scale": 1.0,
            "residuals_mm": dict(zip(sorted(packet["source"]["points"]), residuals.tolist())),
            "rms_residual_mm": float(np.sqrt(np.mean(residuals**2))), "max_residual_mm": maximum,
            "reviewed_limit_mm": float(limit), "fit_pass": True, "canon_effect": "none",
            "anatomically_qualified": False, "fabrication_released": False,
            "limit": "Fit consistency only; landmark authority, anatomy, joint centers and clearance require review"}
