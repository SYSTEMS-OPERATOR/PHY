"""Field trial agent handling onboarding, telemetry and remote stop."""
from __future__ import annotations

import time
import uuid
from dataclasses import dataclass
from typing import Dict

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
    consent_signed: bool = False


class FieldTrialAgent:
    """Manage volunteer onboarding and telemetry."""

    def __init__(self, mqtt_broker: str) -> None:
        self.mqtt_broker = mqtt_broker
        self.volunteers: Dict[str, Volunteer] = {}
        self.client = mqtt.Client()
        self.client.connect(mqtt_broker)

    def onboard_volunteer(self) -> str:
        vid = str(uuid.uuid4())
        self.volunteers[vid] = Volunteer(id=vid, consent_signed=True)
        return vid

    def send_telemetry(self, vid: str, data: dict) -> None:
        topic = f"telemetry/{vid}"
        self.client.publish(topic, str(data), qos=1)

    def red_button(self, vid: str) -> None:
        topic = f"control/{vid}"
        self.client.publish(topic, "KILL", qos=2)
