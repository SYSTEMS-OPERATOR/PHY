from __future__ import annotations

import datetime
from dataclasses import dataclass
from typing import Dict


@dataclass
class Certificate:
    pem: str
    expiry: datetime.datetime

    def is_valid(self) -> bool:
        return self.expiry > datetime.datetime.utcnow()


class SecurityAgent:
    """Handle TLS mutual auth and command signing."""

    def __init__(self, cert: Certificate) -> None:
        self.cert = cert
        self.revoked_ids: set[int] = set()

    def rotate_certificate(self, new_cert: Certificate) -> None:
        self.cert = new_cert

    def validate_command(self, cmd_id: int) -> bool:
        if not self.cert.is_valid():
            raise RuntimeError("certificate expired")
        return cmd_id not in self.revoked_ids

    def whitelist_can_id(self, cmd_id: int) -> None:
        self.revoked_ids.discard(cmd_id)

    def block_can_id(self, cmd_id: int) -> None:
        self.revoked_ids.add(cmd_id)
