from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np
import pybullet as pb


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
