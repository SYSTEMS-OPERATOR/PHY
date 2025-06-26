"""Simulate link failover for CloudTwinSync."""

import time
from cloud.twin_sync import CloudTwinSync
from pathlib import Path


def run_failover(tmp_path: Path) -> float:
    local = tmp_path / "local.json"
    cloud = tmp_path / "cloud.json"
    sync = CloudTwinSync(local, cloud)
    state = {"a": 1}
    sync.step(state)
    sync.rtt_ms = 300
    sync.step({"a": 2})
    time.sleep(0.1)
    sync.rtt_ms = 50
    sync.step({"a": 3})
    sync.resync()
    return sync.last_hash
