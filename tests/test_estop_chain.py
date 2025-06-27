import unittest
import unittest
import time

from hal.hal_driver import GPIOI2CDriver
from safety.safety_controller import SafetyController


class EstopChainTest(unittest.TestCase):
    def test_estop_triggers(self):
        io = GPIOI2CDriver()
        sc = SafetyController(io)
        sc.start()
        io.write_pin(0, 1)
        time.sleep(0.05)
        sc.stop()
        self.assertTrue(sc.estop_state)
        self.assertEqual(io.read_pin(1), 0)


if __name__ == "__main__":
    unittest.main()
