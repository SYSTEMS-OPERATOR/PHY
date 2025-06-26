from __future__ import annotations

"""Cyber security utilities for cloud communication."""

import ssl
import pathlib
from dataclasses import dataclass
from typing import Optional


@dataclass
class CyberSecurityModule:
    cert_path: pathlib.Path
    key_path: pathlib.Path
    ca_path: pathlib.Path
    _context: Optional[ssl.SSLContext] = None

    def build_context(self) -> ssl.SSLContext:
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        if self.ca_path.exists():
            try:
                ctx.load_verify_locations(cafile=str(self.ca_path))
                ctx.verify_mode = ssl.CERT_REQUIRED
            except ssl.SSLError:
                ctx.check_hostname = False
                ctx.verify_mode = ssl.CERT_NONE
        else:
            ctx.check_hostname = False
            ctx.verify_mode = ssl.CERT_NONE
        if self.cert_path.exists() and self.key_path.exists():
            try:
                ctx.load_cert_chain(certfile=str(self.cert_path), keyfile=str(self.key_path))
            except ssl.SSLError:
                pass
        self._context = ctx
        return ctx

    def rotate_certificate(self, new_cert: pathlib.Path, new_key: pathlib.Path) -> None:
        self.cert_path = new_cert
        self.key_path = new_key
        self.build_context()

    def verify(self) -> bool:
        return self._context is not None
