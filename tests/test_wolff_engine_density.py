from skeleton.base import BoneSpec
from adaptation.wolff_engine import WolffAdaptationEngine


def test_wolff_engine_increases_density():
    bone = BoneSpec(
        name="testbone",
        bone_type="long",
        location={},
        articulations=[],
        dimensions={"length_cm": 10.0, "width_cm": 2.0, "thickness_cm": 2.0},
        function=[],
        notable_features=[],
        developmental_notes="",
        variations="",
        unique_id="TEST_BONE",
    )
    bone.set_material("organic")
    engine = WolffAdaptationEngine(bone, baseline_density=bone.material["density"])
    for _ in range(200):
        engine.record(axial_stress=1.2)
    assert bone.material["density"] > engine.baseline_density
