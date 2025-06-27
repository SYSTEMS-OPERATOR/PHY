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

## Privacy
- Encryption with AES-GCM via `privacy.PrivacyManager`.
- DSAR portal located under `web/portal` allows users to submit erase requests.

## Benchmarks
- `bench_pms/ota_latency.py` checks OTA propagation latency.
- `bench_power/thermal_trip.py` simulates thermal runaway response times.
- `bench_privacy/noise_eps.py` validates differential privacy epsilon.
