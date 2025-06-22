from skeleton import load_bones
import json

if __name__ == "__main__":
    bones = load_bones()
    data = {b.unique_id: b.__dict__ for b in bones}
    print(json.dumps(data, indent=2))
