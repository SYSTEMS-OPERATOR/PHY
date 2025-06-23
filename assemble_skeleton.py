from skeleton import load_bones
from skeleton.field import SkeletonField
import json

if __name__ == "__main__":
    bones = load_bones()
    field = SkeletonField(bones)
    data = {b.unique_id: b.self_state() for b in bones}
    print(json.dumps(data, indent=2))
