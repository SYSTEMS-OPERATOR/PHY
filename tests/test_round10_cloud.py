from pathlib import Path
from cloud.twin_sync import CloudTwinSync


def test_cloud_failover(tmp_path):
    local = tmp_path / 'local.json'
    cloud = tmp_path / 'cloud.json'
    sync = CloudTwinSync(local, cloud)
    sync.step({'x': 1})
    sync.rtt_ms = 300
    sync.step({'x': 2})
    sync.rtt_ms = 10
    sync.step({'x': 3})
    assert sync.resync()
