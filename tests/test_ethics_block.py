from ethics.ethics_agent import EthicsAgent


def test_ethics_block():
    agent = EthicsAgent(threshold=0.3)
    risky = agent.assess_risk(0.4, 1.0)
    assert risky
    dangerous = agent.assess_risk(0.5, 0.1)
    assert dangerous
