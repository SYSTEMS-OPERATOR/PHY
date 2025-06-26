from __future__ import annotations

"""Simple benchmark harness."""

import json
import time
from pathlib import Path

from envs.flat_ground import FlatGroundEnv
from envs.stairs import StairsEnv

SUITES = {
    "locomotion": [FlatGroundEnv],
    "stairs": [StairsEnv],
}


def run_suite(name: str, speed: float = 1.0) -> dict:
    env_classes = SUITES.get(name, [])
    results = {}
    for cls in env_classes:
        env = cls()
        start = time.time()
        obs, _ = env.reset()
        done = False
        steps = 0
        while not done and steps < 500:
            obs, _, done, _, _ = env.step([0])
            steps += 1
        duration = time.time() - start
        results[cls.__name__] = duration / max(steps, 1)
    return results


def main(cfg_path: str) -> None:
    cfg = json.loads(Path(cfg_path).read_text())
    out = {}
    for suite in cfg.get("suites", []):
        out[suite] = run_suite(suite, speed=cfg.get("speed", 1.0))
    Path("benchmark_results.json").write_text(json.dumps(out))


if __name__ == "__main__":
    import sys

    main(sys.argv[1])
