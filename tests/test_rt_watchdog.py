import unittest
import time

from control.rt_controller import RTController


class RTWatchdogTest(unittest.TestCase):
    def test_watchdog(self):
        ctrl = RTController(1.0)
        start = time.time()
        ctrl.start(0.01)
        duration = time.time() - start
        self.assertLess(duration, 0.05)


if __name__ == "__main__":
    unittest.main()
