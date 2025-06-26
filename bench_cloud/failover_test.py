import time


def simulate_failover(sync, duration_s: float = 2.0) -> float:
    t0 = time.time()
    while time.time() - t0 < duration_s:
        sync.step()
        time.sleep(0.05)
    return sync.rtt_ms
