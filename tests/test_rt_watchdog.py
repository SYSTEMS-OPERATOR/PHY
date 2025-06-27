from control.rt_controller import RTControllerPy
import time


def test_rt_watchdog():
    ctrl = RTControllerPy(1.0, watchdog_ms=2)
    ctrl.set_desired([0.5])
    ctrl.start()
    time.sleep(0.002)
    ctrl.pause_for(0.006)
    for _ in range(10):
        time.sleep(0.002)
        if ctrl.faulted:
            break
    cmd = ctrl.command[0]
    faulted = ctrl.faulted
    ctrl.stop()
    assert faulted and cmd == 0.0
