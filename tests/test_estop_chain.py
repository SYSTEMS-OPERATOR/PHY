from safety.safety_controller import SafetyController
import time


def test_estop_chain():
    flags = {"estop": False, "torque": 1.0}

    def torque_cb(val: float) -> None:
        flags["torque"] = val

    def estop_cb() -> None:
        flags["estop"] = True

    ctrl = SafetyController(torque_cb, estop_cb)
    ctrl.start()
    ctrl.trigger_estop()
    time.sleep(0.05)
    ctrl.stop()
    assert flags["estop"] and flags["torque"] == 0.0
