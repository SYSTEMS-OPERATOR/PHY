import pytest
from skeleton.bones import load_bones
from skeleton.datasets import load_dataset
from skeleton.field import SkeletonField
from validators.validator_agent import ValidatorAgent


@pytest.fixture(scope="module")
def dataset():
    return load_dataset("female_21_baseline")


@pytest.fixture(scope="module")
def skeleton_field(dataset):
    bones = load_bones("female_21_baseline")
    return SkeletonField(bones)


def test_bone_count(skeleton_field):
    assert len(skeleton_field.bones) >= 190


def test_no_missing_metrics(dataset, skeleton_field):
    for bone in skeleton_field.bones.values():
        if bone.dataset_key:
            metrics = dataset[bone.dataset_key]
            for key in ["length_cm", "mass_g", "density_kg_m3"]:
                assert key in metrics
                if key.endswith("_cm"):
                    assert bone.dimensions.get(key) is not None
                elif key == "mass_g":
                    assert bone.material.get(key) is not None
                else:
                    assert bone.material.get("density") is not None


def test_mass_consistency(dataset, skeleton_field):
    va = ValidatorAgent(skeleton_field, dataset)
    va.run_all_checks()
    # Expect at least one bone to have volume mismatch over threshold
    assert va.results["out_of_range"]


def test_material_switch(skeleton_field, dataset):
    bone = next(b for b in skeleton_field.bones.values() if b.dataset_key)
    bone.set_embodiment("physical")
    original = bone.material.get("density")
    bone.set_material("Ti6Al4V")
    changed = bone.material.get("density")
    bone.set_material("organic")
    reverted = bone.material.get("density")
    bone.set_embodiment("virtual")
    assert changed != original
    assert reverted == original


def test_validator_overall_pass(skeleton_field, dataset):
    va = ValidatorAgent(skeleton_field, dataset)
    va.run_all_checks()
    assert va.results["summary"]["pass"]
