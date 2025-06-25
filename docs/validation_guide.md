# Validation Guide

This repository includes an automated Validation & Reporting Suite (VRS) for
checking bone metrics against the bundled dataset. The suite verifies that each
bone can round‑trip material changes and that mass and volume values remain
consistent.

## Running the validator

```bash
$ python -m eval_skeleton --material organic
$ python -m eval_skeleton --material Ti6Al4V
```

Reports are written to the `reports/` directory and include CSV, JSON and
Markdown summaries.

## Continuous Integration

The GitHub Actions workflow runs the validator for the organic baseline and for
an example synthetic material. Artifacts are uploaded so the reports can be
reviewed from the workflow run.
