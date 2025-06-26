#!/usr/bin/env python
from meta.meta_cognition import MetaCognitionAgent
import time

agent = MetaCognitionAgent()
while True:
    plan = agent.check_anomaly()
    if plan:
        print(plan)
    time.sleep(1)
