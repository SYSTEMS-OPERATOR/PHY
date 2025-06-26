from pathlib import Path
from docs.compliance_doc_gen import ComplianceDocGen
import json


def test_tcf_generation(tmp_path):
    risk = tmp_path / "risk.yaml"
    log = tmp_path / "log.json"
    out = tmp_path / "out.pdf"
    risk.write_text("risk: low")
    log.write_text(json.dumps({"ok": True}))
    gen = ComplianceDocGen(risk, log, out)
    gen.generate()
    assert out.exists()
