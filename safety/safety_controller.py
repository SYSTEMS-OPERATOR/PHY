"""Safety controller implementing a minimal Cat3-style chain."""

from __future__ import annotations

import threading
import time


class SafetyController:
    def __init__(self, io_driver):
        self.io = io_driver
        self.estop_state = False
        self._running = False
        self._thread = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while self._running:
            if self.io.read_pin(0):
                self.trigger_estop()
            time.sleep(0.01)

    def trigger_estop(self):
        self.estop_state = True
        self.io.write_pin(1, 0)  # open contactor mock

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1)

