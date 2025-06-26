from __future__ import annotations

"""Minimal distributed backend wrapping Ray or fallback stubs."""

from dataclasses import dataclass
from typing import Any, Callable

try:
    import ray  # type: ignore
except Exception:  # pragma: no cover - fallback for CI

    class _DummyHandle:
        def __init__(self, cls):
            self.cls = cls

        def remote(self, *args, **kwargs):
            return self.cls(*args, **kwargs)

    class ray:  # type: ignore
        @staticmethod
        def init(address: str | None = None):
            return None

        @staticmethod
        def shutdown():
            return None

        @staticmethod
        def remote(cls):
            return _DummyHandle(cls)

        @staticmethod
        def get(obj):
            return obj


def init(address: str | None = None) -> None:
    """Initialize Ray if available."""
    ray.init(address=address)


def shutdown() -> None:
    """Shutdown runtime."""
    ray.shutdown()


@dataclass
class SimulationWorker:
    """Remote environment worker."""

    env_fn: Callable[[], Any]
    policy_fn: Callable[[], Any]

    def __post_init__(self) -> None:
        self.env = self.env_fn()
        self.policy = self.policy_fn()

    def rollout(self, steps: int = 200):
        obs, _ = self.env.reset()
        data = []
        for _ in range(steps):
            action = self.policy(obs)
            obs, reward, done, _, info = self.env.step(action)
            data.append((obs, action, reward, info))
            if done:
                break
        return data


@dataclass
class ParameterServer:
    """Central store for policy weights."""

    weights: Any

    def get_weights(self) -> Any:
        return self.weights

    def set_gradients(self, grads: Any) -> None:
        if isinstance(self.weights, list) and isinstance(grads, list):
            self.weights = [w + g for w, g in zip(self.weights, grads)]
        else:
            self.weights = grads
