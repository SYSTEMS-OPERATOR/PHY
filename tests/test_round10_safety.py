from safety.safety_controller import SafetyController


def test_estop_chain():
    events = []
    sc = SafetyController(callback=lambda _: events.append('stop'))
    sc.start()
    sc.trigger_estop()
    sc.wait()
    assert sc.contactors_open
    assert 'stop' in events
