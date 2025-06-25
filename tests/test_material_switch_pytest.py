from skvalidators.validator_agent import ValidatorAgent


def test_material_switch(skeleton_field, dataset):
    va = ValidatorAgent(skeleton_field, dataset)
    assert va.check_material_switch() is True
