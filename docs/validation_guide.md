# Validation Guide

This project includes a lightweight Validation & Reporting Suite (VRS) used to
check that the assembled skeleton stays in sync with the reference dataset. The
suite is invoked via the `eval_skeleton` module and produces CSV, JSON and
Markdown reports in the `reports/` directory.

```bash
$ python -m eval_skeleton --material organic
$ python -m eval_skeleton --material Ti6Al4V --export-dir reports/ti
```

The validator performs a series of consistency checks:

* Ensures bones defined in the dataset have required metrics.
* Compares derived volumes against simple geometric volumes.
* Confirms synthetic material tables contain Ti6Al4V, CFRP, PEEK and UHMWPE.
* Verifies that switching materials updates density and mass.

A summary is written to `validation_summary.md` and machine readable data to
`validation_results.json`. The CSV export `bones_metrics.csv` lists one row per
bone with the material currently applied.

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

