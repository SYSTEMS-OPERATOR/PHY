from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np
try:
    import pybullet as pb  # type: ignore
except Exception:  # pragma: no cover - allow tests without pybullet
    class pb:
        ER_TINY_RENDERER = 0

        @staticmethod
        def computeViewMatrix(*args, **kwargs):
            return None

        @staticmethod
        def computeProjectionMatrixFOV(*args, **kwargs):
            return None

        @staticmethod
        def getCameraImage(width, height, **kwargs):
            import numpy as np

            img = np.zeros((height, width, 4), dtype=np.uint8)
            return None, None, img, None, None


@dataclass
class VisionAgent:
    """Minimal vision sensor wrapper around PyBullet."""

    width: int = 64
    height: int = 64

    def capture(self) -> np.ndarray:
        """Return a 64x64 grayscale image in <=8ms."""
        # Use a tiny offscreen render
        view = pb.computeViewMatrix([0, 0, 1], [0, 0, 0], [0, 1, 0])
        proj = pb.computeProjectionMatrixFOV(60, 1.0, 0.1, 10)
        _, _, px, _, _ = pb.getCameraImage(
            self.width,
            self.height,
            viewMatrix=view,
            projectionMatrix=proj,
            renderer=pb.ER_TINY_RENDERER,
        )
        rgb = np.array(px, dtype=np.uint8).reshape(self.height, self.width, 4)
        gray = rgb[:, :, 0].astype(np.float32) / 255.0
        return gray
