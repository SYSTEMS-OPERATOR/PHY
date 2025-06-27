from qms.qms_manager import QMSManager
from pathlib import Path


def test_capa_workflow(tmp_path):
    qms = QMSManager(tmp_path)
    capa = qms.raise_capa("ISSUE-1", "high", "alice")
    assert capa.issue_id == "ISSUE-1"
    qms.close_capa("ISSUE-1")
    assert qms.capa_events["ISSUE-1"].closed is not None
