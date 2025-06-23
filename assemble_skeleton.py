from skeleton import load_field
import json

if __name__ == "__main__":
    field = load_field()
    data = {uid: bone.self_state() for uid, bone in field.bones.items()}
    print(json.dumps(data, indent=2))
