from cloud.twin_sync import CloudTwinSync
import time


def test_cloud_degrade():
    state = {"a": 1}

    def get_local():
        return state

    def send_state(diff):
        pass

    def recv_state():
        time.sleep(0.25)
        return {}

    sync = CloudTwinSync(get_local, send_state, recv_state)
    sync.step()
    assert sync.degraded
