from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

from stable_baselines3.common.monitor import Monitor

from cortex.cortical_agent import CorticalAgent
from envs.flat_ground import FlatGroundEnv
from envs.uneven_ground import UnevenGroundEnv
from envs.stairs import StairsEnv
from envs.ramp import RampEnv
from envs.obstacles import ObstaclesEnv


PHASE_ENVS = [FlatGroundEnv, UnevenGroundEnv, RampEnv, ObstaclesEnv]


def make_env(cls):
    env = cls()
    return Monitor(env)


@dataclass
class CurriculumRunner:
    agent: CorticalAgent

    def run(self, steps_per_phase: int = 1000) -> List[float]:
        success = []
        for phase, env_cls in enumerate(PHASE_ENVS, 1):
            env = make_env(env_cls)
            self.agent.train(env, steps_per_phase)
            obs, _ = env.reset()
            done = False
            passed = True
            while not done:
                action = self.agent.infer(obs)
                obs, _, done, _, _ = env.step(action)
                if abs(obs[0]) > 1.0:
                    passed = False
            success.append(1.0 if passed else 0.0)
        return success
