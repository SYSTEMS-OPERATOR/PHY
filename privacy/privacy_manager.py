"""Privacy module providing encryption and differential privacy."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from cryptography.hazmat.primitives.ciphers.aead import AESGCM
import os
import numpy as np


def add_laplace_noise(value: float, epsilon: float) -> float:
    scale = 1.0 / epsilon
    noise = np.random.laplace(0, scale)
    return value + noise


@dataclass
class PrivacyManager:
    key: bytes

    def __post_init__(self) -> None:
        if len(self.key) != 32:
            raise ValueError("key must be 32 bytes for AES-256-GCM")

    def encrypt(self, data: bytes) -> bytes:
        nonce = os.urandom(12)
        aes = AESGCM(self.key)
        return nonce + aes.encrypt(nonce, data, None)

    def decrypt(self, token: bytes) -> bytes:
        nonce, ct = token[:12], token[12:]
        aes = AESGCM(self.key)
        return aes.decrypt(nonce, ct, None)

    def dp_query(self, value: float, epsilon: float = 1.0) -> float:
        return add_laplace_noise(value, epsilon)
