#!/usr/bin/env python3
from __future__ import annotations

from skeleton.bones import load_field
from skeleton.validation.validator_agent import ValidatorAgent


def main() -> int:
    field = load_field("female_21_baseline")
    validator = ValidatorAgent(field)
    report = validator.run()
    ValidatorAgent.write_reports(report)
    print(f"validation pass={report['summary']['pass']} failed_checks={report['summary']['failed_checks']}")
    return 0 if report["summary"]["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
