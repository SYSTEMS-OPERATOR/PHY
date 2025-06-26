from hal.hal_driver import EtherCATDriver, CANFDDriver


def test_hal_mock():
    eth = EtherCATDriver(2)
    can = CANFDDriver()
    eth.enable()
    eth.set_torque(0, 1.0)
    can.set_torque(1, 2.0)
    assert eth.get_state()[0] == 1.0
    assert can.get_state()[1] == 2.0
