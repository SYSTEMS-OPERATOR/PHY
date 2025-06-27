import subprocess
from pathlib import Path


def test_bms_trip(tmp_path):
    src = Path(__file__).resolve().parents[1] / "power" / "bms_agent.cpp"
    cpp = tmp_path / "test.cpp"
    cpp.write_text(f'#include "{src}"\nint main(){{BMSAgent b(50);b.ingest(60);return 0;}}')
    exe = tmp_path / "prog"
    subprocess.check_call(["g++", str(cpp), "-std=c++17", "-o", str(exe)])
    out = subprocess.check_output([str(exe)], text=True)
    assert "CONTACTOR OPEN" in out
