from pathlib import Path
from security.security_agent import CyberSecurityModule


def test_cert_rotation(tmp_path):
    ca = tmp_path / 'ca.pem'
    cert1 = tmp_path / 'c1.pem'
    key1 = tmp_path / 'k1.pem'
    ca.write_text('ca')
    cert1.write_text('c1')
    key1.write_text('k1')
    sec = CyberSecurityModule(cert1, key1, ca)
    sec.build_context()
    cert2 = tmp_path / 'c2.pem'
    key2 = tmp_path / 'k2.pem'
    cert2.write_text('c2')
    key2.write_text('k2')
    sec.rotate_certificate(cert2, key2)
    assert sec.verify()
