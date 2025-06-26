# Round 10 Hardware Overview

This round introduces a preliminary hardware abstraction layer and real-time
control elements.  The modules are stubs for unit tests but outline the
expected interfaces.

* **HAL** publishes joint and IMU data at 1&nbsp;kHz.
* **RTController** enforces watchdog behaviour.
* **SafetyController** provides a dual-channel E-Stop.
* **CyberSecurityModule** manages certificates.
* **CloudTwinSync** keeps the on-board state in sync with a cloud twin.
