from __future__ import annotations

"""Generate compliance documentation from YAML and test logs."""

import json
from pathlib import Path
from dataclasses import dataclass
from typing import Dict

try:
    import yaml  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    yaml = None


def _simple_yaml(text: str) -> Dict:
    """Parse a minimal subset of YAML for key:value pairs."""
    data: Dict[str, object] = {}
    for line in text.splitlines():
        if ':' not in line:
            continue
        key, value = line.split(':', 1)
        key = key.strip()
        value = value.strip()
        if value.isdigit():
            data[key] = int(value)
        else:
            try:
                data[key] = float(value)
            except ValueError:
                if value.lower() in {'true', 'false'}:
                    data[key] = value.lower() == 'true'
                else:
                    data[key] = value.strip("\"'")
    return data


@dataclass
class ComplianceDocGen:
    risk_yaml: Path
    logs_json: Path
    out_pdf: Path

    def build(self) -> None:
        if self.risk_yaml.exists():
            text = self.risk_yaml.read_text()
            risk = yaml.safe_load(text) if yaml else _simple_yaml(text)
        else:
            risk = {}

        logs = json.loads(self.logs_json.read_text()) if self.logs_json.exists() else {}
        content = f"Risk: {risk}\nLogs: {logs}\n"
        tex_path = self.out_pdf.with_suffix('.tex')
        tex_path.write_text(content)
        self.out_pdf.write_bytes(b"%PDF-1.5 placeholder")
