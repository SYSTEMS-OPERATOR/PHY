from __future__ import annotations

from dataclasses import dataclass, field
from typing import List


@dataclass
class AlignmentSafetyLayer:
    """Basic intent filter and stop mechanism."""

    unsafe_words: List[str] = field(default_factory=lambda: ["kill", "attack"])
    stop: bool = False

    def check_intent(self, text: str) -> bool:
        words = text.lower().split()
        return not any(w in words for w in self.unsafe_words)

    def trigger_stop(self) -> None:
        self.stop = True
