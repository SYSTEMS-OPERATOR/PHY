"""Python wrapper for RT control logic used in unit tests."""

import time


class RTController:
    def __init__(self, limit: float):
        self.limit = limit
        self.running = False

    def start(self, duration: float):
        self.running = True
        end = time.time() + duration
        while self.running and time.time() < end:
            time.sleep(0.001)
        self.running = False

    def stop(self):
        self.running = False
