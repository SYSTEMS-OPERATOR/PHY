from skvalidators.validator_agent import ValidatorAgent


def test_bone_count_warning(skeleton_field, dataset):
    va = ValidatorAgent(skeleton_field, dataset)
    va.check_bone_count()
    assert "bone_count" in va.results
    assert va.results["bone_count"] == len(skeleton_field.bones)
