import unittest

from security.security_agent import SecurityAgent


class CertRotationTest(unittest.TestCase):
    def test_expired_cert(self):
        agent = SecurityAgent([])
        agent.cert_expiry = agent.cert_expiry.replace(year=2000)
        self.assertFalse(agent.cert_valid())
        agent.rotate_cert(1)
        self.assertTrue(agent.cert_valid())


if __name__ == "__main__":
    unittest.main()
