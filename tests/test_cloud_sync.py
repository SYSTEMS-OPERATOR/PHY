import unittest

from cloud.twin_sync import CloudTwinSync


class CloudSyncTest(unittest.TestCase):
    def test_state_hash_changes(self):
        sync = CloudTwinSync()
        state = {"a": 1}
        h1 = sync.diff_and_send(state)
        state["a"] = 2
        h2 = sync.diff_and_send(state)
        self.assertNotEqual(sync.last_hash, str(hash(h1)))
        self.assertNotEqual(h1, h2)


if __name__ == "__main__":
    unittest.main()
