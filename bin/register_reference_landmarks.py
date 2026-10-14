#!/usr/bin/env python3
"""Fit explicitly annotated reference landmarks without changing PHY geometry."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from skeleton.reference_registration import register_landmarks
from skeleton.exporters.exporter_agent import json_bytes
from skeleton.validation.dimension_evidence import _unique_object


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--input", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    temp = None
    try:
        raw = args.input.read_bytes()
        packet = json.loads(raw, object_pairs_hook=_unique_object)
        result = register_landmarks(packet)
        result["input_packet_sha256"] = hashlib.sha256(raw).hexdigest()
        content = json_bytes(result)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(dir=args.output.parent, delete=False) as out:
            temp = Path(out.name);out.write(content)
        temp.replace(args.output)
        print(json.dumps({"fit_pass": True, "fabrication_released": False, "max_residual_mm": result["max_residual_mm"]}))
        return 0
    except (ValueError, TypeError, KeyError, OSError) as error:
        print(json.dumps({"fit_pass": False, "issue": str(error)}))
        return 1
    finally:
        if temp is not None:
            temp.unlink(missing_ok=True)


if __name__ == "__main__":
    raise SystemExit(main())
