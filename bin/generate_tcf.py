#!/usr/bin/env python
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parent.parent / "docs"))
from compliance_doc_gen import ComplianceDocGen
import json


def main() -> None:
    gen = ComplianceDocGen(Path("risk.yaml"), Path("test_log.json"), Path("TCF.pdf"))
    # placeholder test log
    Path("test_log.json").write_text(json.dumps({"tests": "ok"}))
    Path("risk.yaml").write_text("risk: low")
    gen.generate()
    print("tcf generated")


if __name__ == "__main__":
    main()
