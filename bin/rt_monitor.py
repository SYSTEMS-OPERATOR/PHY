#!/usr/bin/env python
"""Simple jitter monitor for RTController."""

import time
from control.rt_controller import RTController


def main() -> None:
    rt = RTController()
    times = []

    def cb(_):
        times.append(time.time())

    rt.add_callback(cb)
    rt.start()
    time.sleep(0.01)
    rt.stop()
    deltas = [1000 * (t2 - t1) for t1, t2 in zip(times, times[1:])]
    print({"jitter_us": deltas})


if __name__ == "__main__":
    main()
