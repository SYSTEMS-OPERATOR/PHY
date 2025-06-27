import unittest

from hal.hal_driver import EtherCATDriver, CANFDDriver, GPIOI2CDriver, HAL


class HALMockTest(unittest.TestCase):
    def test_drivers_basic(self):
        motor = EtherCATDriver()
        io = GPIOI2CDriver()
        hal = HAL(motor)
        motor.enable()
        motor.set_torque(0.5)
        dropped = hal.start(duration=0.01)
        hal.stop()
        self.assertFalse(motor.get_state()["torque"] == 0)
        self.assertGreaterEqual(dropped, 0)


if __name__ == "__main__":
    unittest.main()
