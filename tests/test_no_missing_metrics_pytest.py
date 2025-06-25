from skvalidators.validator_agent import ValidatorAgent


def test_no_missing_metrics(skeleton_field, dataset):
    va = ValidatorAgent(skeleton_field, dataset)
    assert va.check_required_metrics() is True
