from __future__ import annotations

"""Lifelong learning pipeline with simple replay buffer."""

from dataclasses import dataclass, field
from typing import Any, Deque, Tuple
import os
import pickle
from collections import deque


@dataclass
class LifelongLearner:
    agent: Any
    buffer_path: str = "replay.pkl"
    max_size: int = 10000
    replay: Deque[Tuple] = field(default_factory=lambda: deque(maxlen=10000))

    def add_experience(self, exp: Tuple) -> None:
        self.replay.append(exp)
        with open(self.buffer_path, "wb") as f:
            pickle.dump(list(self.replay), f)

    def load_replay(self) -> None:
        if os.path.exists(self.buffer_path):
            with open(self.buffer_path, "rb") as f:
                data = pickle.load(f)
                self.replay = deque(data, maxlen=self.max_size)

    def consolidate(self) -> None:
        # very small EWC-like regularization
        if not self.replay:
            return
        w = self.agent.get_weights()
        penalty = sum(sum(abs(x)) for x in self.replay) / len(self.replay)
        new_w = [wi - 1e-4 * penalty for wi in w]
        self.agent.set_weights(new_w)

    def evaluate(self, env_fn, steps: int = 200) -> float:
        env = env_fn()
        obs, _ = env.reset()
        total = 0.0
        for _ in range(steps):
            act = self.agent.act(obs)
            obs, reward, done, _, _ = env.step(act)
            total += reward
            if done:
                break
        return total / steps
