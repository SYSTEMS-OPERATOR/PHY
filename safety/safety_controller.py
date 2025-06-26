from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Callable


@dataclass
class SafetyController:
    """Dual-threaded safety controller stub."""

    torque_cb: Callable[[float], None]
    estop_cb: Callable[[], None]
    faulted: bool = field(init=False, default=False)
    _threads: list[threading.Thread] = field(init=False, default_factory=list)
    _running: bool = field(init=False, default=False)

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._threads = [threading.Thread(target=self._plc_loop, daemon=True),
                         threading.Thread(target=self._micro_loop, daemon=True)]
        for t in self._threads:
            t.start()

    def stop(self) -> None:
        self._running = False
        for t in self._threads:
            t.join()
        self._threads.clear()

    def trigger_estop(self) -> None:
        self.faulted = True
        self.estop_cb()
        self.torque_cb(0.0)

    # mocked loops
    def _plc_loop(self) -> None:
        while self._running:
            time.sleep(0.01)
            if self.faulted:
                self.torque_cb(0.0)

    def _micro_loop(self) -> None:
        while self._running:
            time.sleep(0.02)
            if self.faulted:
                self.estop_cb()
