import time
from hal.hal_driver import EtherCATDriver, IMUDriver, HALPublisher

class DummyIMU(IMUDriver):
    def get_gyro_accel(self):
        return {'x': 0.0, 'y': 0.0, 'z': 0.0}

    def get_temp(self):
        return 25.0

def test_hal_mock():
    motor = EtherCATDriver()
    imu = DummyIMU()
    motor.enable()
    motor.set_torque('j1', 0.5)
    pub = HALPublisher(motor, imu)
    data = []
    pub.subscribe(lambda s, i: data.append((s, i)))
    pub.start()
    time.sleep(0.002)
    pub.stop()
    assert data and data[0][0]['enabled'] == 1.0
