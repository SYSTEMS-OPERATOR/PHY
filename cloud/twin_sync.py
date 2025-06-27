"""Minimal cloud twin synchronisation example."""

from __future__ import annotations

import json
import time


class CloudTwinSync:
    def __init__(self):
        self.last_hash = ""
        self.local_state = {}

    def diff_and_send(self, cloud_state: dict) -> bytes:
        payload = json.dumps(cloud_state).encode()
        self.last_hash = str(hash(payload))
        return payload

    def run(self, get_state_func, send_func, duration=1.0):
        end = time.time() + duration
        while time.time() < end:
            state = get_state_func()
            data = self.diff_and_send(state)
            send_func(data)
            time.sleep(1 / 60)
