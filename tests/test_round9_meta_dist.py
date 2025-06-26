import os
import shutil
import json
import pytest
from pathlib import Path

from meta.meta_cognition import MetaCognitionAgent
from training.distributed_ppo import TrainerDistPPO
from training.lifelong_learner import LifelongLearner
from envs.base_env import BaseEnv


class DummyPolicy:
    def __init__(self):
        self.w = [0.0]

    def act(self, obs):
        return [0.0]

    def get_weights(self):
        return list(self.w)

    def set_weights(self, w):
        self.w = list(w)


def test_meta_detects_collapse():
    agent = MetaCognitionAgent(reward_window=10)
    for _ in range(20):
        agent.record_step(0.0)
    plan = agent.check_anomaly()
    assert plan and plan["action"] == "rollback"


def test_dist_rollout_consistency():
    policy = DummyPolicy()
    trainer1 = TrainerDistPPO(BaseEnv, policy, num_workers=1)
    trainer2 = TrainerDistPPO(BaseEnv, policy, num_workers=2)
    roll1 = trainer1.collect_rollouts(5)
    roll2 = trainer2.collect_rollouts(5)
    s1 = sum(len(r) for r in roll1)
    s2 = sum(len(r) for r in roll2)
    assert abs(s1 - s2) <= 5


def test_lifelong_buffer(tmp_path):
    policy = DummyPolicy()
    learner = LifelongLearner(policy, buffer_path=str(tmp_path / "buf.pkl"))
    learner.add_experience((1, 2, 3))
    learner.load_replay()
    assert learner.replay


@pytest.mark.skipif(shutil.which("docker") is None, reason="no docker")
def test_docker_build(tmp_path):
    shutil.copytree("docker", tmp_path / "d")
    cmd = f"docker build -f {tmp_path/'d/Dockerfile'} {tmp_path/'d'}"
    assert os.system(cmd) == 0


def test_benchmark_smoke(tmp_path):
    from benchmarks.benchmark_runner import main

    cfg = {"suites": ["locomotion"], "speed": 1.0}
    p = tmp_path / "cfg.json"
    p.write_text(json.dumps(cfg))
    main(str(p))
    assert (Path("benchmark_results.json")).exists()

from dist import backend


def test_backend_stub():
    backend.init()
    backend.shutdown()
