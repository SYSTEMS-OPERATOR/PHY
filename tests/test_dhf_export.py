from qms.qms_manager import QMSManager
from pathlib import Path


def test_dhf_export(tmp_path):
    qms = QMSManager(tmp_path)
    qms.add_document('sop.md', 'alice')
    qms.raise_capa('ISSUE', 'low', 'bob')
    dhf = qms.export_dhf()
    assert any('sop.md' in line for line in dhf)
    assert any('CAPA ISSUE' in line for line in dhf)
