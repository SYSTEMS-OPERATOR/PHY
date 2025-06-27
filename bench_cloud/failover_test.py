"""Simulate link loss and local autonomy check."""

import time


def run_check(sim_fn, duration=2.0):
    t_end = time.time() + duration
    lost = False
    while time.time() < t_end:
        if not sim_fn():
            lost = True
    return lost

