#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
from qms.qms_manager import QMSManager


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("issue")
    parser.add_argument("severity")
    parser.add_argument("owner")
    args = parser.parse_args()

    qms = QMSManager(Path("."))
    capa = qms.raise_capa(args.issue, args.severity, args.owner)
    print(f"CAPA raised: {capa.issue_id} -> {capa.owner}")


if __name__ == "__main__":
    main()
