#!/usr/bin/env python
"""Toggle E-Stop using the SafetyController."""

import argparse
from hal.hal_driver import GPIOI2CDriver
from safety.safety_controller import SafetyController


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("action", choices=["engage", "release"])
    args = parser.parse_args()

    io = GPIOI2CDriver()
    sc = SafetyController(io)
    if args.action == "engage":
        io.write_pin(0, 1)
        sc.trigger_estop()
    else:
        io.write_pin(0, 0)
    print(f"estop_state={sc.estop_state}")


if __name__ == "__main__":
    main()
