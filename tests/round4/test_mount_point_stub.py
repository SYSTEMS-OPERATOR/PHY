from skeleton.base import BoneSpec


def test_mount_point_field():
    b = BoneSpec(name='X', bone_type='long', location={}, articulations=[], dimensions={'length_cm':1,'width_cm':1,'thickness_cm':1}, function=[], notable_features=[], developmental_notes='', variations='', unique_id='X')
    assert isinstance(b.mount_points, list)
