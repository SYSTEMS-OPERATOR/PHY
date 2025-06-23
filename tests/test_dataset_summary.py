import os
import sys
import json
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from skeleton.bones import load_bones
from skeleton.field import SkeletonField

if __name__ == "__main__":
    bones = load_bones("female_21_baseline")
    field = SkeletonField(bones)
    summary = {b.unique_id: b.export() for b in bones}
    report = {
        "health": field.health(),
        "faults": field.faults(),
        "metrics_audit": field.audit_metrics(),
        "bones": summary,
    }
    print(json.dumps(report, indent=2))
