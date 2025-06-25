import pytest

from skeleton.bones import load_bones
from skeleton.datasets import load_dataset
from validators.validator_agent import ValidatorAgent


@pytest.fixture(scope="module")
def dataset():
    return load_dataset("female_21_baseline")


@pytest.fixture(scope="module")
def skeleton_fixture(dataset):
    bones = load_bones("female_21_baseline")
    for b in bones:
        b.dataset = dataset
        b.apply_dataset(dataset)
    return bones


def test_bone_count(skeleton_fixture):
    assert len(skeleton_fixture) == 193


def test_no_missing_metrics(skeleton_fixture):
    for bone in skeleton_fixture:
        if bone.dataset_key is None:
            continue
        metrics = bone.validate_metrics()
        assert metrics == {}


def test_mass_consistency(skeleton_fixture, dataset):
    bone = next(b for b in skeleton_fixture if b.dataset_key)
    metrics = dataset[bone.dataset_key]
    if all(k in metrics for k in ("mass_g", "density_kg_m3")):
        vol_geo = bone.volume_cm3()
        if vol_geo is not None:
            derived = (metrics["mass_g"] / 1000) / metrics["density_kg_m3"] * 1e6
            diff = abs(vol_geo - derived) / derived
            assert diff <= 25


def test_material_switch(skeleton_fixture):
    bone = next(b for b in skeleton_fixture if b.dataset_key)
    bone.set_embodiment("physical", {"density": bone.material.get("density", 1800)})
    bone.set_material("organic")
    base = bone.material["density"]
    bone.set_material("Ti6Al4V")
    ti = bone.material["density"]
    bone.set_material("organic")
    assert ti != base and bone.material["density"] == base


def test_validator_overall_pass(skeleton_fixture, dataset):
    va = ValidatorAgent(skeleton_fixture, dataset)
    va.run_all_checks()
    assert va.results["summary"]["pass"] is True

