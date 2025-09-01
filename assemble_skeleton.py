from skeleton.agents.assembler import AssemblerAgent
from pathlib import Path
import json

if __name__ == "__main__":
    sk = AssemblerAgent()
    sk.discover_bones()       # reads skeleton/bones/*.yml
    sk.assemble()
    report = sk.validate()
    out = Path("dist"); out.mkdir(exist_ok=True, parents=True)
    (out / "skeleton.json").write_text(sk.to_json())
    print("integrity:", json.dumps(report))
