from __future__ import annotations

"""Hardware Abstraction Layer for physical actuators and sensors."""

import threading
import time
from dataclasses import dataclass, field
from typing import Dict, Iterable


class MotorDriver:
    """Abstract motor driver interface."""

    def enable(self) -> None:
        raise NotImplementedError

    def disable(self) -> None:
        raise NotImplementedError

    def set_torque(self, joint_id: str, value: float) -> None:
        raise NotImplementedError

    def get_state(self) -> Dict[str, float]:
        """Return current state such as torque and temperature."""
        raise NotImplementedError


class IMUDriver:
    """Abstract IMU driver."""

    def get_gyro_accel(self) -> Dict[str, float]:
        raise NotImplementedError

    def get_temp(self) -> float:
        raise NotImplementedError


class IODriver:
    """Generic GPIO/I2C interface."""

    def read_pin(self, ident: int) -> int:
        raise NotImplementedError

    def write_pin(self, ident: int, val: int) -> None:
        raise NotImplementedError


@dataclass
class EtherCATDriver(MotorDriver):
    """Simple placeholder EtherCAT driver."""

    enabled: bool = False
    torque_commands: Dict[str, float] = field(default_factory=dict)

    def enable(self) -> None:
        self.enabled = True

    def disable(self) -> None:
        self.enabled = False
        self.torque_commands.clear()

    def set_torque(self, joint_id: str, value: float) -> None:
        self.torque_commands[joint_id] = float(value)

    def get_state(self) -> Dict[str, float]:
        return {"enabled": float(self.enabled)} | self.torque_commands


@dataclass
class CANFDDriver(MotorDriver):
    """Placeholder CAN‑FD driver."""

    bus_id: int = 0
    torque_commands: Dict[str, float] = field(default_factory=dict)

    def enable(self) -> None:
        pass

    def disable(self) -> None:
        self.torque_commands.clear()

    def set_torque(self, joint_id: str, value: float) -> None:
        self.torque_commands[joint_id] = float(value)

    def get_state(self) -> Dict[str, float]:
        return self.torque_commands.copy()


@dataclass
class GPIOI2CDriver(IODriver):
    """I2C expander GPIO driver."""

    pins: Dict[int, int] = field(default_factory=dict)

    def read_pin(self, ident: int) -> int:
        return int(self.pins.get(ident, 0))

    def write_pin(self, ident: int, val: int) -> None:
        self.pins[ident] = int(val)


class HALPublisher:
    """Publishes joint and IMU states at 1 kHz."""

    def __init__(self, motor: MotorDriver, imu: IMUDriver) -> None:
        self.motor = motor
        self.imu = imu
        self.running = False
        self.thread: threading.Thread | None = None
        self.subscribers: list = []

    def _loop(self) -> None:
        period = 0.001
        while self.running:
            state = self.motor.get_state()
            imu = self.imu.get_gyro_accel() | {"temp": self.imu.get_temp()}
            for cb in list(self.subscribers):
                cb(state, imu)
            time.sleep(period)

    def start(self) -> None:
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._loop, daemon=True)
            self.thread.start()

    def stop(self) -> None:
        self.running = False
        if self.thread:
            self.thread.join(timeout=1)
            self.thread = None

    def subscribe(self, cb) -> None:
        self.subscribers.append(cb)
