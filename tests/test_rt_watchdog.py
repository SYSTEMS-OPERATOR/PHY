from control.rt_controller import RTControllerPy
import time


def test_rt_watchdog():
    ctrl = RTControllerPy(1.0)
    ctrl.set_desired([0.5])
    ctrl.start()
    time.sleep(0.003)
    cmd = ctrl.command[0]
    ctrl.stop()
    assert abs(cmd - 0.5) < 1e-6
