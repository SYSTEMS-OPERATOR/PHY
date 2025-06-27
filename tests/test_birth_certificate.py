from manufacturing.cal_rig import CalibrationRig
from pathlib import Path


def test_birth_certificate(tmp_path):
    rig = CalibrationRig("SN123")
    rig.calibrate()
    rig.upload(tmp_path)
    cert = (tmp_path / "SN123.json").read_text()
    assert "SN123" in cert
