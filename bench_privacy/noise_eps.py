"""Benchmark differential privacy epsilon compliance."""
from __future__ import annotations

from privacy.privacy_manager import add_laplace_noise


def check_epsilon(value: float, epsilon: float) -> float:
    noisy = [add_laplace_noise(value, epsilon) for _ in range(1000)]
    mean_noise = sum(noisy) / len(noisy) - value
    return mean_noise


if __name__ == "__main__":
    eps = 1.0
    delta = check_epsilon(10.0, eps)
    print(f"epsilon={eps} mean_noise={delta:.4f}")
