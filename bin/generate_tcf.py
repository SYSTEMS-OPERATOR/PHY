#!/usr/bin/env python
"""Package supplied evidence, or explicitly exercise the generator with fixtures."""

import argparse
import json
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from docs.compliance_doc_gen import ComplianceDocGen


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate a TCF stub packet from existing risk.yaml and test_log.json.",
    )
    parser.add_argument(
        "--smoke", action="store_true",
        help="Use isolated synthetic inputs; no tests are run or qualified.",
    )
    args = parser.parse_args()
    output = Path("TCF.pdf")

    try:
        if args.smoke:
            with tempfile.TemporaryDirectory(prefix="phy-tcf-smoke-") as directory:
                root = Path(directory)
                risk, log = root / "risk.yaml", root / "test_log.json"
                risk.write_text("evidence_kind: synthetic_smoke_fixture\nrisk: unassessed\n")
                log.write_text(json.dumps({
                    "evidence_kind": "synthetic_smoke_fixture", "tests": "not_run",
                }))
                ComplianceDocGen(risk, log, output).generate()
        else:
            risk, log = Path("risk.yaml"), Path("test_log.json")
            missing = [str(path) for path in (risk, log) if not path.is_file()]
            if missing:
                parser.error("missing evidence input(s): " + ", ".join(missing))
            ComplianceDocGen(risk, log, output).generate()
    except (OSError, json.JSONDecodeError) as error:
        parser.exit(1, f"TCF generation failed: {error}\n")

    print("synthetic TCF smoke packet generated; no tests run" if args.smoke
          else "TCF stub packet generated from supplied evidence")


if __name__ == "__main__":
    main()
