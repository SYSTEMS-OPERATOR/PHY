from skeleton import load_bones
from skeleton.field import SkeletonField
import json


if __name__ == "__main__":
    bones = load_bones("female_21_baseline")
    field = SkeletonField(bones)
    data = {
        "bones": {b.unique_id: b.self_state() for b in bones},
        "health": field.health(),
        "faults": field.faults(),
    }    
    print(json.dumps(data, indent=2))
