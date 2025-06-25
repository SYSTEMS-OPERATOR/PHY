from soft.muscle_spec import MuscleSpec
from soft.muscle_agent import MuscleAgent


def test_muscle_generates_force():
    spec = MuscleSpec(
        name="biceps",
        origin={"bone_uid": "A", "point": "p1"},
        insertion={"bone_uid": "B", "point": "p2"},
        max_isometric_force_N=1000.0,
        optimal_fiber_len_cm=5.0,
        tendon_slack_len_cm=2.0,
    )
    muscle = MuscleAgent(spec)
    muscle.attach_to_bones(lambda: 5.0, lambda: 0.0)
    f = muscle.update(0.01, activation=0.5)
    assert f > 0
