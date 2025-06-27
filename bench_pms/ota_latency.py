"""Benchmark OTA latency for patch distribution."""
from __future__ import annotations

import random
import time


def measure_latency(patch_size_mb: float) -> float:
    """Simulate latency proportional to patch size."""
    base = 3600.0  # seconds for 1 MB baseline
    noise = random.uniform(-0.1, 0.1) * base
    return max(base * patch_size_mb + noise, 0.0)


if __name__ == "__main__":
    size = 2.0
    start = time.time()
    latency = measure_latency(size)
    elapsed = time.time() - start
    print(f"patch_size={size}MB latency={latency:.2f}s compute_time={elapsed:.2f}s")
