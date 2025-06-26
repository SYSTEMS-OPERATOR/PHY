from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Any, Dict


@dataclass
class CloudTwinSync:
    """Synchronise state with cloud twin."""

    get_local_state: callable
    send_state: callable
    recv_state: callable
    last_hash: str = field(init=False, default="")
    rtt_ms: float = field(init=False, default=0.0)

    def step(self) -> None:
        start = time.time()
        local = self.get_local_state()
        remote = self.recv_state()
        diff = {
            k: v for k, v in local.items() if remote.get(k) != v
        }
        if diff:
            self.send_state(diff)
        self.rtt_ms = (time.time() - start) * 1000
        self.last_hash = str(hash(json.dumps(local, sort_keys=True)))
