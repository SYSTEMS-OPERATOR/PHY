from __future__ import annotations

from dataclasses import dataclass
from typing import List

from training.curriculum_runner import CurriculumRunner
from cortex.cortical_agent import CorticalAgent
from tasks.cooperative_tasks import PassTheCubeEnv
from tasks.competitive_tasks import TugOfWarEnv


PHASES = [PassTheCubeEnv, TugOfWarEnv]


@dataclass
class CurriculumSocialRunner(CurriculumRunner):
    """Curriculum with social phases."""

    def run(self, steps_per_phase: int = 100) -> List[float]:
        results = []
        for env_cls in PHASES:
            env = env_cls()
            self.agent.train(env, steps_per_phase)
            obs, _ = env.reset()
            done = False
            success = False
            while not done:
                action = {"agent_0": self.agent.infer(obs)}
                obs, reward, done, _, _ = env.step(action)
                if reward > 0:
                    success = True
            results.append(1.0 if success else 0.0)
        return results
