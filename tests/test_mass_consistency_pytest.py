from skvalidators.validator_agent import ValidatorAgent


def test_mass_consistency(skeleton_field, dataset):
    va = ValidatorAgent(skeleton_field, dataset)
    va.check_volume_consistency()
    assert isinstance(va.results.get("out_of_range"), dict)
