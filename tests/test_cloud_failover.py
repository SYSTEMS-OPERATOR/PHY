from cloud.twin_sync import CloudTwinSync


def test_cloud_failover():
    state = {"a": 1}

    def get_local():
        return state

    def send_state(diff):
        state.update(diff)

    def recv_state():
        return {}

    sync = CloudTwinSync(get_local, send_state, recv_state)
    sync.step()
    assert sync.last_hash
