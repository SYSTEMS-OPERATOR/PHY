from skvalidators.validator_agent import ValidatorAgent


def test_validator_overall_pass(skeleton_field, dataset):
    va = ValidatorAgent(skeleton_field, dataset)
    results = va.run_all_checks()
    assert results["summary"]["pass"] is True
