"""Optional symbolic overlay utilities.

These helpers are intentionally non-fabrication-critical.
"""

from __future__ import annotations

from typing import Dict


def soul_annotation(intent: str) -> Dict[str, str]:
    return {
        "intent": intent,
        "layer": "soul",
        "note": "Symbolic context only; no physical parameters modified.",
    }
