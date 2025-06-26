from pathlib import Path
from docs.compliance_doc_gen import ComplianceDocGen


def test_docgen(tmp_path):
    risk = tmp_path / 'risk.yaml'
    logs = tmp_path / 'logs.json'
    out = tmp_path / 'tcf.pdf'
    risk.write_text('a: 1')
    logs.write_text('{"t":1}')
    gen = ComplianceDocGen(risk, logs, out)
    gen.build()
    assert out.exists()
