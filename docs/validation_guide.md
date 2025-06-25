# Validation Guide

This project includes a validation & reporting suite (VRS) for the digital skeleton.

## Usage

Run the evaluator with the desired material:

```bash
python -m eval_skeleton --material organic
python -m eval_skeleton --material Ti6Al4V
```

Reports are saved under `./reports/` and include CSV, JSON and Markdown summaries.

To run the full test suite:

```bash
pytest
```

