import json
import random
from pathlib import Path


def run_trace(out: str) -> None:
    data = [random.uniform(-1, 1) for _ in range(100)]
    Path(out).write_text(json.dumps(data))


if __name__ == "__main__":
    run_trace("force_trace.json")
