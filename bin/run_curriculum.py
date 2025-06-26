#!/usr/bin/env python
from __future__ import annotations

import sys
from pathlib import Path

from cortex.cortical_agent import CorticalAgent
from training.curriculum_runner import CurriculumRunner
from envs.base_env import BaseEnv


def main() -> None:
    agent = CorticalAgent(BaseEnv().observation_space, BaseEnv().action_space)
    runner = CurriculumRunner(agent)
    success = runner.run(steps_per_phase=100)
    print("phase successes", success)


if __name__ == "__main__":
    main()
