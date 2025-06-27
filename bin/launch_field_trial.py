#!/usr/bin/env python3
from __future__ import annotations

from field.field_trial_agent import FieldTrialAgent


def main() -> None:
    agent = FieldTrialAgent("localhost")
    vid = agent.onboard_volunteer()
    agent.send_telemetry(vid, {"hello": "world"})
    agent.red_button(vid)


if __name__ == "__main__":
    main()
