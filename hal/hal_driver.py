"""Hardware Abstraction Layer drivers.

Provides simple interfaces for motor, IMU and IO drivers. Adapters can
be swapped depending on the bus in use (EtherCAT, CAN‑FD, GPIO/I2C).
The base classes implement the minimum API required by the control
stack. Actual hardware specific implementations should override these
methods.
"""

from __future__ import annotations

import time
from typing import Tuple


class MotorDriver:
    """Abstract motor driver."""

    def enable(self) -> None:
        raise NotImplementedError

    def disable(self) -> None:
        raise NotImplementedError

    def set_torque(self, value: float) -> None:
        raise NotImplementedError

    def get_state(self) -> dict:
        """Return current joint state dictionary."""
        raise NotImplementedError


class IMUDriver:
    """Abstract IMU driver."""

    def get_gyro_accel(self) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        raise NotImplementedError

    def get_temp(self) -> float:
        raise NotImplementedError


class IODriver:
    """Abstract digital IO driver."""

    def read_pin(self, pin_id: int) -> int:
        raise NotImplementedError

    def write_pin(self, pin_id: int, value: int) -> None:
        raise NotImplementedError


# --- Adapters ---------------------------------------------------------------
class EtherCATDriver(MotorDriver):
    """Mock EtherCAT driver for unit tests."""

    def __init__(self) -> None:
        self.torque = 0.0
        self.enabled = False

    def enable(self) -> None:
        self.enabled = True

    def disable(self) -> None:
        self.enabled = False

    def set_torque(self, value: float) -> None:
        self.torque = value

    def get_state(self) -> dict:
        return {"enabled": self.enabled, "torque": self.torque}


class CANFDDriver(MotorDriver):
    """Mock CAN‑FD driver."""

    def __init__(self) -> None:
        self.torque = 0.0
        self.enabled = False

    def enable(self) -> None:
        self.enabled = True

    def disable(self) -> None:
        self.enabled = False

    def set_torque(self, value: float) -> None:
        self.torque = value

    def get_state(self) -> dict:
        return {"enabled": self.enabled, "torque": self.torque}


class GPIOI2CDriver(IODriver):
    """Mock GPIO/I2C driver."""

    def __init__(self) -> None:
        self.pins = {}

    def read_pin(self, pin_id: int) -> int:
        return self.pins.get(pin_id, 0)

    def write_pin(self, pin_id: int, value: int) -> None:
        self.pins[pin_id] = value


# --- ROS2 Publishing -------------------------------------------------------
class HAL:
    """Simplified HAL publisher running at 1 kHz."""

    def __init__(self, motor: MotorDriver, imu: IMUDriver | None = None) -> None:
        self.motor = motor
        self.imu = imu
        self.running = False

    def start(self, duration: float = 1.0) -> None:
        """Start publishing mock joint and IMU states."""
        self.running = True
        end_time = time.time() + duration
        interval = 0.001
        next_time = time.time()
        dropped = 0
        while self.running and time.time() < end_time:
            if time.time() >= next_time:
                # Here we would publish to ROS2; for tests we just sleep
                self.motor.get_state()
                if self.imu:
                    self.imu.get_gyro_accel()
                next_time += interval
            else:
                # frame missed
                dropped += 1
                next_time += interval
            time.sleep(max(0.0, next_time - time.time()))
        self.running = False
        return dropped

    def stop(self) -> None:
        self.running = False

