"""Calibration rig controlling UR cobot and vision system."""
from __future__ import annotations

import json
from pathlib import Path


class CalibrationRig:
    def __init__(self, serial: str) -> None:
        self.serial = serial
        self.results = {}

    def calibrate(self) -> None:
        # Placeholder for robotic calibration
        self.results = {"torque": 1.0, "sensor": self.serial}

    def birth_certificate(self) -> str:
        firmware_hash = "deadbeef"  # placeholder
        data = {
            "serial": self.serial,
            "results": self.results,
            "firmware": firmware_hash,
        }
        return json.dumps(data)

    def upload(self, qms_path: Path) -> None:
        cert = self.birth_certificate()
        (qms_path / f"{self.serial}.json").write_text(cert)
