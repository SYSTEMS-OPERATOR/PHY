#!/usr/bin/env python3
import os, re, json, pathlib, hashlib, csv
ROOT = pathlib.Path(__file__).resolve().parents[1]
EXCLUDE = {".git", ".github", "dist", "__pycache__", ".venv", "envs", "data", "export"}
INFO = []

def sniff_imports(path):
    if path.suffix not in {".py", ".ipynb"}: return []
    try: text = path.read_text(errors="ignore")
    except Exception: return []
    mods = re.findall(r'^\s*(?:from\s+([\w\.]+)\s+import|import\s+([\w\.]+))', text, re.M)
    flat = [ (a or b).split('.')[0] for a,b in mods ]
    return sorted(set([m for m in flat if m]))

def lines_of_code(path):
    try: return sum(1 for _ in path.open(errors="ignore"))
    except Exception: return 0

for p in ROOT.rglob("*"):
    if any(part in EXCLUDE for part in p.parts): continue
    if p.is_file():
        rel = p.relative_to(ROOT).as_posix()
        kind = p.suffix.lower() or "dirless"
        INFO.append({
            "path": rel,
            "suffix": kind,
            "bytes": p.stat().st_size,
            "loc": lines_of_code(p),
            "imports": sniff_imports(p),
        })

(out_csv := ROOT/"repo_map.csv").write_text("")
with open(out_csv, "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=INFO[0].keys())
    w.writeheader(); w.writerows(INFO)

by_dir = {}
for row in INFO:
    top = row["path"].split("/")[0]
    agg = by_dir.setdefault(top, {"files":0,"loc":0,"bytes":0})
    agg["files"] += 1; agg["loc"] += row["loc"]; agg["bytes"] += row["bytes"]

lines = ["# META_REPO_MAP", "", "| dir | files | loc | bytes |", "|-|-:|-:|-:|"]
for d,agg in sorted(by_dir.items()):
    lines.append(f"| `{d}` | {agg['files']:,} | {agg['loc']:,} | {agg['bytes']:,} |")
(ROOT/"META_REPO_MAP.md").write_text("\n".join(lines))
print("wrote:", out_csv, "and META_REPO_MAP.md")
