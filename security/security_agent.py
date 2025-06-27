"""Cyber security module with placeholder TLS and CAN filtering."""

from __future__ import annotations

import datetime


class SecurityAgent:
    def __init__(self, allowed_can_ids=None):
        self.allowed_can_ids = set(allowed_can_ids or [])
        self.cert_expiry = datetime.datetime.utcnow() + datetime.timedelta(days=1)

    def validate_can_id(self, msg_id: int) -> bool:
        return msg_id in self.allowed_can_ids

    def cert_valid(self) -> bool:
        return datetime.datetime.utcnow() < self.cert_expiry

    def rotate_cert(self, days: int = 1):
        self.cert_expiry = datetime.datetime.utcnow() + datetime.timedelta(days=days)
