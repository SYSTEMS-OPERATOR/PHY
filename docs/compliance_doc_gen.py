"""Generate a simple technical construction file PDF."""

from __future__ import annotations

import json
from pathlib import Path


def generate_tcf(risk_yaml: str, test_log: str, output: Path) -> Path:
    """Combine risk and test logs into a markdown report."""
    data = {
        "risk": risk_yaml,
        "test_log": test_log,
    }
    output.write_text(json.dumps(data, indent=2))
    return output


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("risk", help="risk yaml")
    parser.add_argument("log", help="pytest json log")
    parser.add_argument("-o", "--output", default="tcf.json")
    args = parser.parse_args()
    generate_tcf(args.risk, args.log, Path(args.output))
