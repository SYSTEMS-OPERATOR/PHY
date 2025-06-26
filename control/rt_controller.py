from __future__ import annotations

"""Python wrapper for RTController."""

import threading
import time
from typing import Callable, List


class RTController:
    def __init__(self) -> None:
        self.desired = 0.0
        self.torque_limit = 1.0
        self.callbacks: List[Callable[[float], None]] = []
        self.running = False
        self.thread: threading.Thread | None = None

    def set_desired(self, tau: float) -> None:
        self.desired = tau

    def add_callback(self, cb: Callable[[float], None]) -> None:
        self.callbacks.append(cb)

    def start(self) -> None:
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.running = False
        if self.thread:
            self.thread.join()
            self.thread = None

    def _loop(self) -> None:
        period = 0.001
        while self.running:
            start = time.time()
            cmd = max(-self.torque_limit, min(self.desired, self.torque_limit))
            for cb in list(self.callbacks):
                cb(cmd)
            dt = time.time() - start
            if dt > 0.002:
                for cb in list(self.callbacks):
                    cb(0.0)
            sleep = period - (time.time() - start)
            if sleep > 0:
                time.sleep(sleep)
