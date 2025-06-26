#!/usr/bin/env python
"""Send an E-Stop toggle via SafetyController."""
import argparse
from safety.safety_controller import SafetyController


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("action", choices=["trigger", "clear"])
    args = parser.parse_args()

    def noop(*_):
        pass

    ctrl = SafetyController(noop, noop)
    ctrl.start()
    if args.action == "trigger":
        ctrl.trigger_estop()
        print("estop triggered")
    ctrl.stop()


if __name__ == "__main__":
    main()
