#!/usr/bin/env python
from safety.alignment_layer import AlignmentSafetyLayer


def main() -> None:
    layer = AlignmentSafetyLayer()
    layer.trigger_stop()
    print("stop", layer.stop)


if __name__ == "__main__":
    main()
