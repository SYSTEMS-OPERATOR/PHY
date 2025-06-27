from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Dict, Iterable, Tuple, Callable


class MotorDriver:
    """Abstract motor driver interface."""

    def enable(self) -> None:
        pass

    def disable(self) -> None:
        pass

    def set_torque(self, joint_id: int, torque: float) -> None:
        pass

    def get_state(self) -> Dict[int, float]:
        return {}


class IMUDriver:
    """Abstract IMU driver."""

    def get_gyro_accel(self) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)

    def get_temp(self) -> float:
        return 25.0


class IODriver:
    """General purpose IO driver."""

    def read_pin(self, pin_id: int) -> int:
        return 0

    def write_pin(self, pin_id: int, val: int) -> None:
        pass


class EtherCATDriver(MotorDriver):
    """Stub EtherCAT driver for tests."""

    def __init__(self, joints: int) -> None:
        self._state = {i: 0.0 for i in range(joints)}
        self.enabled = False

    def enable(self) -> None:
        self.enabled = True

    def disable(self) -> None:
        self.enabled = False

    def set_torque(self, joint_id: int, torque: float) -> None:
        self._state[joint_id] = float(torque)

    def get_state(self) -> Dict[int, float]:
        return dict(self._state)


class CANFDDriver(MotorDriver, IODriver):
    """Stub CAN-FD driver."""

    def __init__(self) -> None:
        self._pins: Dict[int, int] = {}
        self._state: Dict[int, float] = {}

    def enable(self) -> None:
        pass

    def disable(self) -> None:
        pass

    def set_torque(self, joint_id: int, torque: float) -> None:
        self._state[joint_id] = float(torque)

    def get_state(self) -> Dict[int, float]:
        return dict(self._state)

    def read_pin(self, pin_id: int) -> int:
        return self._pins.get(pin_id, 0)

    def write_pin(self, pin_id: int, val: int) -> None:
        self._pins[pin_id] = int(val)


class GPIOI2CDriver(IODriver):
    """Stub GPIO driver via I2C."""

    def __init__(self) -> None:
        self._pins: Dict[int, int] = {}

    def read_pin(self, pin_id: int) -> int:
        return self._pins.get(pin_id, 0)

    def write_pin(self, pin_id: int, val: int) -> None:
        self._pins[pin_id] = int(val)


@dataclass
class HAL:
    motor: MotorDriver
    imu: IMUDriver
    io: IODriver
    publish_cb: Callable[[Dict[int, float], Tuple[float, float, float], Tuple[float, float, float]], None]
    running: bool = field(init=False, default=False)
    thread: threading.Thread | None = field(init=False, default=None)
    frames: int = field(init=False, default=0)
    dropped: int = field(init=False, default=0)

    def _loop(self) -> None:
        next_time = time.time()
        while self.running:
            now = time.time()
            if now - next_time > 0.0015:
                self.dropped += 1
                next_time = now
            joint_state = self.motor.get_state()
            gyro, accel = self.imu.get_gyro_accel()
            self.publish_cb(joint_state, gyro, accel)
            self.frames += 1
            next_time += 0.001
            sleep = max(0.0, next_time - time.time())
            time.sleep(sleep)

    def start(self) -> None:
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.running = False
        if self.thread:
            self.thread.join()
            self.thread = None
