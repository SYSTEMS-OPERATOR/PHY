from hal.hal_driver import HAL, EtherCATDriver, IMUDriver, IODriver
import time


class DummyIMU(IMUDriver):
    def get_gyro_accel(self):
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)


class DummyIO(IODriver):
    pass


def test_hal_rate():
    driver = EtherCATDriver(1)
    imu = DummyIMU()
    io = DummyIO()
    records = []

    def cb(state, gyro, accel):
        records.append(state)

    hal = HAL(driver, imu, io, cb)
    hal.start()
    time.sleep(0.01)
    hal.stop()
    assert hal.frames > 5
    assert hal.dropped <= hal.frames * 0.2
