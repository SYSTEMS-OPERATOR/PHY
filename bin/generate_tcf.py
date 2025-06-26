#!/usr/bin/env python
"""Run the compliance document generator."""

from pathlib import Path
from docs.compliance_doc_gen import ComplianceDocGen


def main() -> None:
    gen = ComplianceDocGen(Path('risk.yaml'), Path('test_logs.json'), Path('tcf.pdf'))
    gen.build()
    print('Generated', gen.out_pdf)


if __name__ == '__main__':
    main()
