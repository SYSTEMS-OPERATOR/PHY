# Round 10 Hardware Overview

This document sketches the real hardware transition for the **Vitruvian
Armature**.  The focus of Round 10 is moving from simulation into a physical
prototype while preserving tight realtime guarantees.

## Wiring Diagram

```
Motor <-> EtherCAT <-> Jetson
IMU   <-> CAN‑FD   <-> Jetson
IO    <-> GPIO/I2C <-> Jetson
```

## Realtime Budget

| Stage          | Target µs |
|----------------|-----------|
| Sensor read    | 200       |
| Control calc   | 300       |
| Actuator write | 200       |
| Safety margin  | 200       |

## Risk Analysis Template

```
hazard:
  description: example pinch point
  severity: 4
  occurrence: 1
  mitigation: dual channel estop
```
