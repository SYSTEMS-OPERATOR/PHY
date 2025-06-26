import datetime
import pytest
from security.security_agent import SecurityAgent, Certificate


def test_cert_rotation():
    cert = Certificate(pem="A", expiry=datetime.datetime.utcnow() - datetime.timedelta(days=1))
    agent = SecurityAgent(cert)
    with pytest.raises(RuntimeError):
        agent.validate_command(1)
    new_cert = Certificate(pem="B", expiry=datetime.datetime.utcnow() + datetime.timedelta(days=1))
    agent.rotate_certificate(new_cert)
    assert agent.validate_command(1)
