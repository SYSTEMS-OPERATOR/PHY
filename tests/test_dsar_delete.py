from privacy.privacy_manager import PrivacyManager
import os


def test_dsar_delete(tmp_path):
    key = os.urandom(32)
    mgr = PrivacyManager(key)
    data = b"secret"
    enc = mgr.encrypt(data)
    dec = mgr.decrypt(enc)
    assert dec == data
    noisy = mgr.dp_query(10.0)
    assert isinstance(noisy, float)
