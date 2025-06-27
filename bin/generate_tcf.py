#!/usr/bin/env python
from pathlib import Path
import json
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

from docs.compliance_doc_gen import ComplianceDocGen


def main() -> None:
    gen = ComplianceDocGen(Path("risk.yaml"), Path("test_log.json"), Path("TCF.pdf"))
    # placeholder test log
    Path("test_log.json").write_text(json.dumps({"tests": "ok"}))
    Path("risk.yaml").write_text("risk: low")
    gen.generate()
    print("tcf generated")


if __name__ == "__main__":
    main()
