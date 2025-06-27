from pms.pms_pipeline import PMSPipeline


def test_pms_anomaly():
    pms = PMSPipeline()
    for i in range(10):
        pms.ingest(1.0)
    pms.train()
    assert not pms.detect(1.0)
