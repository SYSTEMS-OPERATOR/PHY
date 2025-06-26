from __future__ import annotations

"""Simplified distributed PPO trainer using the dist backend."""

from dataclasses import dataclass
from typing import Any, Callable, List

import numpy as np

from dist import backend


@dataclass
class TrainerDistPPO:
    """Coordinate multiple workers and update shared policy."""

    env_fn: Callable[[], Any]
    policy: Any
    num_workers: int = 1
    async_mode: bool = False

    def __post_init__(self) -> None:
        backend.init()
        self.server = backend.ParameterServer(self.policy.get_weights())
        worker_cls = backend.ray.remote(backend.SimulationWorker)
        self.workers = [
            worker_cls.remote(self.env_fn, self.policy.act)
            for _ in range(self.num_workers)
        ]

    def collect_rollouts(self, steps: int) -> List[Any]:
        futures = [w.rollout.remote(steps) for w in self.workers]
        return backend.ray.get(futures)

    def update_policy(self, rollouts: List[Any]) -> None:
        grads = [0.0 for _ in self.policy.get_weights()]
        for data in rollouts:
            for _, act, _, _ in data:
                for i, a in enumerate(np.atleast_1d(act)):
                    grads[i] += a
        grads = [g / len(rollouts) for g in grads]
        self.server.set_gradients(grads)
        self.policy.set_weights(self.server.get_weights())

    def train(self, iterations: int = 1, steps_per_rollout: int = 200) -> None:
        for _ in range(iterations):
            rollouts = self.collect_rollouts(steps_per_rollout)
            self.update_policy(rollouts)

