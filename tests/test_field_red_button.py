from field import field_trial_agent


def test_field_red_button(monkeypatch):
    sent = {}

    class FakeClient:
        def publish(self, topic, payload, qos=0):
            sent['topic'] = topic
            sent['payload'] = payload

        def connect(self, broker):
            pass

    monkeypatch.setattr(field_trial_agent.mqtt, "Client", lambda: FakeClient())
    agent = field_trial_agent.FieldTrialAgent("local")
    vid = agent.onboard_volunteer()
    agent.red_button(vid)
    assert sent['payload'] == "KILL"

