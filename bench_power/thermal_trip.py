"""Simulate thermal runaway detection time."""
from __future__ import annotations

import time
from power.bms_agent import BMSAgent


def simulate_trip(temp_rate: float, duration: float, limit: float = 60.0) -> float:
    """Run BMSAgent with rising temperature."""
    agent = BMSAgent(limit)
    temp = 25.0
    start = time.time()
    step = duration / 100
    for _ in range(100):
        temp += temp_rate * step
        agent.ingest(temp)
        if temp > limit:
            break
        time.sleep(step)
    return time.time() - start


if __name__ == "__main__":
    t = simulate_trip(5.0, 0.2)
    print(f"trip_time={t:.3f}s")
