#!/usr/bin/env python
import json
from benchmarks.benchmark_runner import main

CONFIG = {"suites": ["locomotion"], "speed": 5.0}

if __name__ == "__main__":
    with open("bench_cfg.json", "w") as f:
        json.dump(CONFIG, f)
    main("bench_cfg.json")
