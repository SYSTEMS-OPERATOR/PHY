from cloud.twin_sync import CloudTwinSync
import time


def test_cloud_failover():
    state = {"a": 1}

    def get_local():
        return state

    def send_state(diff):
        state.update(diff)

    def recv_state():
        return {}

    def recv_state_delayed():
        time.sleep(0.25)
        return {}

    sync = CloudTwinSync(get_local, send_state, recv_state)
    sync.step()
    assert not sync.degraded
    # simulate high latency
    sync.recv_state = recv_state_delayed
    sync.step()
    assert sync.degraded
