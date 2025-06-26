#!/usr/bin/env python
"""Command line E-Stop trigger."""

from safety.safety_controller import SafetyController


def main() -> None:
    sc = SafetyController()
    sc.start()
    input("Press Enter to trigger E-Stop...")
    sc.trigger_estop()
    sc.wait()
    print("E-Stop engaged", sc.contactors_open)


if __name__ == "__main__":
    main()
