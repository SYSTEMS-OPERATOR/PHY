from __future__ import annotations

"""Synchronises on-board state with cloud twin."""

import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict

import zlib


def _hash(data: Dict) -> int:
    return zlib.adler32(json.dumps(data, sort_keys=True).encode())


@dataclass
class CloudTwinSync:
    local_path: Path
    cloud_path: Path
    rtt_ms: int = 0
    last_hash: int = field(default=0, init=False)

    def step(self, state: Dict) -> None:
        new_hash = _hash(state)
        if self.rtt_ms > 200:
            return  # degraded mode
        cloud_state = json.loads(self.cloud_path.read_text()) if self.cloud_path.exists() else {}
        if _hash(cloud_state) != new_hash:
            self.cloud_path.write_text(json.dumps(state))
        self.last_hash = new_hash

    def resync(self) -> bool:
        if not self.cloud_path.exists():
            return False
        cloud_state = json.loads(self.cloud_path.read_text())
        self.local_path.write_text(json.dumps(cloud_state))
        self.last_hash = _hash(cloud_state)
        return True
