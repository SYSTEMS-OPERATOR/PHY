from __future__ import annotations

from dataclasses import dataclass, field
from typing import List

from affect.emotion_agent import EmotionAgent


@dataclass
class DialogueManager:
    """Very small context-based dialogue manager."""

    emotion_agent: EmotionAgent
    history: List[str] = field(default_factory=list)
    max_history: int = 5

    def get_response(self, text: str) -> str:
        self.history.append(text)
        self.history = self.history[-self.max_history :]
        # simple rule-based reply
        if "hello" in text.lower():
            reply = "Hello there!"
        else:
            reply = "Acknowledged."
        return reply
