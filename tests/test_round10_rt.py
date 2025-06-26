import time
from control.rt_controller import RTController


def test_rt_watchdog():
    rt = RTController()
    outputs = []
    def cb(t):
        outputs.append(t)
        if len(outputs) == 1:
            time.sleep(0.005)
    rt.add_callback(cb)
    rt.set_desired(0.8)
    rt.start()
    time.sleep(0.005)
    rt.stop()
    assert outputs[0] > 0.0
    assert 0.0 in outputs
