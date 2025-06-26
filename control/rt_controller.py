from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import List


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


@dataclass
class RTControllerPy:
    torque_limit: float
    watchdog_ms: int = 2
    command: List[float] = field(default_factory=list)
    desired: List[float] = field(default_factory=list)
    running: bool = field(init=False, default=False)
    _thread: threading.Thread | None = field(init=False, default=None)

    def start(self) -> None:
        if self.running:
            return
        self.running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self.running = False
        if self._thread:
            self._thread.join()
            self._thread = None

    def set_desired(self, torques: List[float]) -> None:
        self.desired = list(torques)

    def _loop(self) -> None:
        next_time = time.time()
        while self.running:
            next_time += 0.001
            self.command = [clamp(t, self.torque_limit) for t in self.desired]
            time.sleep(max(0.0, next_time - time.time()))

