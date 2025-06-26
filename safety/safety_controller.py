from __future__ import annotations

"""Safety controller implementing a dual-channel E-Stop."""

import threading
import time
from dataclasses import dataclass, field
from typing import Callable


@dataclass
class SafetyController:
    callback: Callable[[bool], None] | None = None
    estop: bool = False
    contactors_open: bool = False
    _threads: list[threading.Thread] = field(default_factory=list, init=False)

    def _plc_thread(self) -> None:
        while not self.estop:
            time.sleep(0.01)
        self.contactors_open = True
        if self.callback:
            self.callback(True)

    def _micro_thread(self) -> None:
        while not self.estop:
            time.sleep(0.01)
        self.contactors_open = True

    def start(self) -> None:
        self.estop = False
        self.contactors_open = False
        self._threads = [
            threading.Thread(target=self._plc_thread, daemon=True),
            threading.Thread(target=self._micro_thread, daemon=True),
        ]
        for t in self._threads:
            t.start()

    def trigger_estop(self) -> None:
        self.estop = True

    def wait(self, timeout: float = 0.2) -> None:
        start = time.time()
        for t in self._threads:
            remaining = timeout - (time.time() - start)
            if remaining > 0:
                t.join(remaining)
