"""Field trial agent handling onboarding, telemetry and remote stop."""
from __future__ import annotations

import time
import uuid
from dataclasses import dataclass
from typing import Dict

try:
    from cryptography.hazmat.primitives.asymmetric.ed25519 import (
        Ed25519PrivateKey,
        Ed25519PublicKey,
    )
    from cryptography.hazmat.primitives import serialization
except Exception:  # pragma: no cover - optional dependency
    Ed25519PrivateKey = None  # type: ignore
    Ed25519PublicKey = None  # type: ignore
    class _DummySerialization:
        class Encoding:
            Raw = None

        class PublicFormat:
            Raw = None

    serialization = _DummySerialization()

try:
    import paho.mqtt.client as mqtt
except Exception:  # pragma: no cover - optional dependency
    class _DummyMQTT:
        class Client:
            def connect(self, _):
                pass

            def publish(self, *args, **kwargs):
                pass

    mqtt = _DummyMQTT()


@dataclass
class Volunteer:
    id: str
    pubkey: bytes
    consent_signed: bool = False


class FieldTrialAgent:
    """Manage volunteer onboarding and telemetry."""

    def __init__(self, mqtt_broker: str) -> None:
        self.mqtt_broker = mqtt_broker
        self.volunteers: Dict[str, Volunteer] = {}
        proto = getattr(mqtt, "MQTTv5", None)
        self.client = mqtt.Client(protocol=proto) if proto else mqtt.Client()
        self.client.connect(mqtt_broker)
        self._signer = Ed25519PrivateKey.generate() if Ed25519PrivateKey else None

    def onboard_volunteer(self) -> str:
        """Register a volunteer with simulated FIDO2 consent."""
        vid = str(uuid.uuid4())
        # In a real system we would verify a FIDO2 signature here. For
        # demonstration we sign the volunteer ID with our private key and store
        # the public key for later verification.
        if self._signer:
            signature = self._signer.sign(vid.encode())
            pubkey = self._signer.public_key().public_bytes(
                serialization.Encoding.Raw,
                serialization.PublicFormat.Raw,
            )
        else:  # fallback if cryptography not installed
            signature = vid.encode()
            pubkey = b""
        self.volunteers[vid] = Volunteer(id=vid, pubkey=pubkey, consent_signed=True)
        # record signature via MQTT for audit
        self.client.publish(f"consent/{vid}", signature.hex(), qos=1)
        return vid

    def send_telemetry(self, vid: str, data: dict) -> None:
        """Send anonymised telemetry payload."""
        topic = f"telemetry/{vid}"
        self.client.publish(topic, str(data), qos=1)

    def red_button(self, vid: str) -> None:
        """Send a high QoS kill command."""
        topic = f"control/{vid}"
        self.client.publish(topic, "KILL", qos=2)
