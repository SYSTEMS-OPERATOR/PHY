# Round 11 Field Trials & QMS

This document outlines the basic processes for the pilot field trials and the accompanying Quality Management System (QMS).

## QMS
- Controlled documents stored in Git with signed commits.
- CAPA workflow managed via `qms.qms_manager.QMSManager`.
- Design History File exported as a text log linking requirements to commits.

## Field Trials
- Volunteers sign electronic consent via FIDO2 mobile app.
- Telemetry is streamed to a secure endpoint.
- A remote "red button" can stop the device over MQTT.

## Ethics & IRB
- The EthicsAgent evaluates risk vs. benefit and records adverse events.

## Post-Market Surveillance
- Anomaly detection halts OTA roll-outs and raises CAPA entries automatically.
