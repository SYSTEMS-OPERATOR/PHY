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
    faulted: bool = field(init=False, default=False)
    _thread: threading.Thread | None = field(init=False, default=None)
    _pause_s: float = field(init=False, default=0.0)
    _fault_until: float = field(init=False, default=0.0)

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

    def pause_for(self, seconds: float) -> None:
        self._pause_s = seconds

    def _loop(self) -> None:
        next_time = time.time()
        while self.running:
            next_time += 0.001
            if self._pause_s:
                time.sleep(self._pause_s)
                self._pause_s = 0.0
            now = time.time()
            latency = (now - next_time + 0.001) * 1000
            if latency > self.watchdog_ms:
                self.command = [0.0 for _ in self.desired]
                self.faulted = True
                self._fault_until = time.time() + 0.002
            elif self._fault_until > time.time():
                self.command = [0.0 for _ in self.desired]
                self.faulted = True
            else:
                self.command = [clamp(t, self.torque_limit) for t in self.desired]
                self.faulted = False
            time.sleep(max(0.0, next_time - time.time()))
