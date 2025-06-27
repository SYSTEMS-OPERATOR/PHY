#!/usr/bin/env python
"""Run compliance doc generation."""

import argparse
from pathlib import Path
from docs.compliance_doc_gen import generate_tcf


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("risk")
    parser.add_argument("log")
    parser.add_argument("-o", "--output", default="tcf.json")
    args = parser.parse_args()
    out = generate_tcf(args.risk, args.log, Path(args.output))
    print(f"TCF generated at {out}")


if __name__ == "__main__":
    main()
